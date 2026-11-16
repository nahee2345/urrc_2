#!/usr/bin/env python3
"""URRC v16 vehicle preflight (read-only to participants and world assets).

STATIC_PASS is not a guarantee of successful qualifying, semi or final driving.
Optional --live samples an already-running competition via v15; it does not
spawn cars, publish commands or modify submitted code.
"""
from __future__ import annotations
import argparse
from datetime import datetime
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET

TEAM_IDS = tuple(f'team_{i:02d}' for i in range(1, 13))
TRACKS = ('monza','spa','monaco','cota','hungaroring','baku')
REQUIRED = ('entry_id','ros_package','algorithm_launch_file',
            'vehicle_model_file','speed_topic','steering_topic','lidar_topic')


def safe_model(folder, name):
    if not isinstance(name,str) or not name:
        raise ValueError('vehicle_model_file 누락')
    path=(folder/name).resolve()
    if not path.is_relative_to(folder.resolve()):
        raise ValueError('차량 모델 파일 경로가 참가 폴더 밖을 가리킵니다')
    if not path.is_file():
        raise ValueError(f'차량 모델 파일 없음: {name}')
    return path


def check_entries(root, target=None, *, conversion=False):
    from base_link_check_v6 import check as base_check
    from inspection_v4 import inspect_model
    folder=root/'src/team'
    reports=[]
    rows={}
    for team in TEAM_IDS:
        if target and team!=target:
            continue
        location=folder/team
        manifest=location/'race_entry.json'
        if not manifest.is_file():
            reports.append({'team':team,'status':'EMPTY' if location.is_dir() else 'MISSING',
                'errors':[],'warnings':[],'checks':{}})
            continue
        errors=[];warnings=[];checks={}
        entry=None
        try:
            entry=json.loads(manifest.read_text(encoding='utf-8'))
            if not isinstance(entry,dict):
                raise ValueError('race_entry.json은 JSON 객체여야 합니다')
            for key in REQUIRED:
                if not isinstance(entry.get(key),str) or not entry[key].strip():
                    errors.append(f'필수 항목 누락 또는 형식 오류: {key}')
            if entry.get('entry_id')!=team:
                errors.append(f"entry_id 불일치: {entry.get('entry_id')!r}, 폴더={team}")
            model_path=None
            try:
                model_path=safe_model(location, entry.get('vehicle_model_file'))
                checks['model_path']=str(model_path.relative_to(location))
            except ValueError as exc:errors.append(str(exc))
            packages={}
            for xml in sorted(location.rglob('package.xml')):
                if any(part in ('.git','build','install','log') for part in xml.relative_to(location).parts):
                    continue
                try:
                    name=ET.parse(xml).getroot().findtext('name')
                    if name:packages[name]=xml
                except ET.ParseError as exc:errors.append(f'package.xml 파싱 실패: {xml.relative_to(location)}: {exc}')
            checks['ros_packages']=sorted(packages)
            if not packages:
                errors.append('ROS 패키지 package.xml 없음')
            pkg=entry.get('ros_package')
            if pkg and pkg not in packages:
                errors.append(f'ros_package={pkg!r}와 실제 package.xml 이름이 다릅니다: {sorted(packages)}')
            launch=entry.get('algorithm_launch_file')
            if launch:
                matches=list(location.rglob('launch/'+launch))
                matches=[x for x in matches if 'build' not in x.parts and 'install' not in x.parts]
                if not matches:
                    errors.append(f'알고리즘 launch 파일 없음: {launch}')
                else:
                    checks['launch_files']=[str(p.relative_to(location)) for p in matches]
                    try:
                        text='\n'.join(p.read_text(encoding='utf-8') for p in matches)
                        if 'use_sim_time' not in text:
                            warnings.append('launch 파일에서 use_sim_time 설정을 확인하지 못했습니다')
                        if 'race_start_topic' not in text and '/race/start/' not in text:
                            warnings.append('launch 파일에서 대회 출발 신호 구독 연결을 확인하지 못했습니다')
                    except UnicodeError:
                        warnings.append('launch 소스 인코딩 확인 필요')
            fmt=str(entry.get('vehicle_model_format', model_path.suffix.lstrip('.') if model_path else 'sdf')).lower()
            if fmt not in ('sdf','urdf'):
                errors.append(f'지원하지 않는 차량 모델 형식: {fmt}')
            if model_path and fmt in ('sdf','urdf'):
                try:
                    e,w=inspect_model(model_path,fmt,entry.get('base_link_name','base_link'))
                    errors.extend(e)
                    warnings.extend(s for s in w if s!='자동 PASS는 전체 부피/센서 위치 적합성을 보증하지 않습니다')
                except (ET.ParseError, ValueError, TypeError, OSError) as exc:
                    errors.append(f'차량 모델 구조 검사 실패: {exc}')
                try:
                    result=base_check(model_path,entry.get('base_link_name','base_link'))
                    checks['base_link']={'status':result['status'],
                        'center_m':result.get('collision_center_relative_to_base_m')}
                    errors.extend('base_link: '+x for x in result.get('errors',[]))
                    warnings.extend('base_link: '+x for x in result.get('warnings',[]))
                except (ET.ParseError, ValueError, TypeError, OSError) as exc:
                    errors.append(f'base_link 검사 실패: {exc}')
                try:
                    model=ET.parse(model_path).getroot()
                    types={s.get('type','').lower() for s in model.findall('.//sensor')}
                    if not any('camera' in ty or 'rgb' in ty for ty in types):
                        warnings.append('모델 XML에 카메라 센서 선언을 찾지 못했습니다. 대회 camera 출발은 실시간 영상 확인 필요')
                    if not any('lidar' in ty or 'ray' in ty for ty in types):
                        warnings.append('모델 XML에 LiDAR 센서 선언을 찾지 못했습니다. 실제 ROS 수신 검증 필요')
                except (ET.ParseError,OSError):pass
                if conversion:
                    if shutil.which('gz'):
                        try:
                            p=subprocess.run(['gz','sdf','-p',str(model_path)],
                                capture_output=True,text=True,timeout=20)
                            if p.returncode:
                                errors.append('Gazebo SDF 변환 실패: '+(p.stderr or p.stdout).strip()[-600:])
                            else:
                                doc=ET.fromstring(p.stdout)
                                links={x.get('name') for x in doc.findall('.//model/link')}
                                if entry.get('base_link_name','base_link') not in links:
                                    errors.append('Gazebo 변환 후 base_link 링크가 사라졌습니다. URDF 병합/모델 구조 확인 필요')
                                checks['gazebo_sdf_conversion']='PASS' if entry.get('base_link_name','base_link') in links else 'FAIL'
                        except (OSError,subprocess.TimeoutExpired,ET.ParseError) as exc:
                            errors.append(f'Gazebo 모델 변환 실행 오류: {exc}')
                    else:
                        warnings.append('gz 명령 없음: Gazebo 모델 변환 미검증')
            for key in ('speed_topic','steering_topic','lidar_topic'):
                name=entry.get(key)
                if isinstance(name,str) and name and not name.startswith('/'):
                    warnings.append(f'{key} 절대 토픽명이 아닙니다: {name}')
            # The official race runner reads race/entrants, not src/team.
            # Warn when the live intake hasn't been refreshed; never sync implicitly.
            official=root/'race/entrants'
            if official.is_dir():
                imported=official/team/'race_entry.json'
                if not imported.is_file():
                    warnings.append('공식 접수본 없음: bash race_v16.sh sync 필요')
                else:
                    try:
                        official_entry=json.loads(imported.read_text(encoding='utf-8'))
                        changed=[k for k in REQUIRED if official_entry.get(k)!=entry.get(k)]
                        if changed:
                            errors.append('원본과 공식 접수본 설정 불일치 ('+', '.join(changed)+'): sync 후 재검수 필요')
                    except (OSError,ValueError,TypeError) as exc:
                        errors.append(f'공식 접수본 읽기 오류: {exc}')
            rows[team]=entry
        except (OSError,ValueError,TypeError) as exc:
            errors.append(f'race_entry.json 읽기 실패: {exc}')
        reports.append({'team':team,'status':'FAIL' if errors else 'REVIEW' if warnings else 'STATIC_PASS',
            'errors':sorted(set(errors)), 'warnings':sorted(set(warnings)), 'checks':checks})
    # Detect namespace/topic collisions across team packages.
    used={}
    for team,entry in rows.items():
        for field in ('ros_package','speed_topic','steering_topic','lidar_topic'):
            value=entry.get(field)
            if not isinstance(value,str) or not value:
                continue
            label=(field,value) if field=='ros_package' else ('topic',value)
            used.setdefault(label,[]).append((team,field))
    byteam={r['team']:r for r in reports}
    for (kind,value),references in used.items():
        teams={team for team,_ in references}
        if len(teams)>1:
            for team in teams:
                byteam[team]['errors'].append(f'다른 참가팀과 {kind} 중복: {value}; 관련 팀={sorted(teams)}')
                byteam[team]['status']='FAIL'
    for r in reports:
        r['errors']=sorted(set(r['errors']))
    return reports


def summarize_live(snapshot, team):
    """Assess observed boundaries, never infer a lap-completion guarantee."""
    entry=snapshot.get('teams',{}).get(team,{})
    samples=snapshot.get('samples',{})
    findings=[]
    def add(status,code,detail):findings.append({'status':status,'area':code,'detail':detail})
    if not snapshot.get('world_match'):
        add('FAIL','world','대회 Gazebo 월드/파티션 일치가 확인되지 않았습니다')
    clock=samples.get('/clock',{})
    duration=(clock.get('max_value') or 0)-(clock.get('min_value') or 0)
    if clock.get('count',0)<2 or duration<=0:
        add('FAIL','clock','/clock 수신 또는 시뮬레이션 시간 진행이 확인되지 않았습니다')
    gate=samples.get(f'/race/start/{team}',{})
    if not gate.get('count',0):
        add('REVIEW','gate','관측 시간 안에 출발 신호 수신 없음(아직 출발 전일 수 있음)')
    elif gate.get('last_value') is not True:
        add('REVIEW','gate','최종 출발 허가 true를 관측하지 못함(출발 전일 수 있음)')
    pose=samples.get(f'/model/{team}/urrc_timing',{})
    if not pose.get('count',0):
        add('FAIL','timing','중앙 base_link 계측 포즈 미수신')
    related=[]
    from environment_diagnostic_v15 import topic_is_team
    for topic, s in samples.items():
        if topic_is_team(topic,team,entry):
            related.append((topic,s))
    for kind in ('camera','lidar'):
        seen=[(t,s) for t,s in related if s.get('kind')==kind and s.get('count',0)>0]
        if not seen:
            add('FAIL',kind,f'ROS {kind} 센서 메시지 수신 없음')
        else:
            for topic,s in seen:
                lag=s.get('stamp_lag_sim_seconds')
                if isinstance(lag,(int,float)) and (lag>1.0 or lag < -0.2):
                    add('REVIEW',kind+'_time',f'{topic} 시간차={lag:.3f}초: 동기화 확인 필요')
                g=snapshot.get('ros_topics',{}).get(topic,{})
                if g.get('qos_state')=='incompatible':
                    add('FAIL',kind+'_qos',f'{topic}: 발행·구독 QoS 비호환')
                elif not g.get('subscribers_excluding_diagnostic',0):
                    add('REVIEW',kind+'_consumer',f'{topic}: 참가 노드 구독 여부 확인 필요')
    speed=samples.get(entry.get('speed_topic'),{})
    steer=samples.get(entry.get('steering_topic'),{})
    if not speed.get('count',0) or not steer.get('count',0):
        add('REVIEW','commands','속도/조향 명령이 관측되지 않았습니다. 출발 이후 추가 확인 필요')
    elif gate.get('last_value') is True and speed.get('max_abs_value',0)<=0.05:
        add('REVIEW','commands','출발 허가 이후에도 속도 명령이 0에 가깝습니다. 카메라·안전조건 확인 필요')
    if not any(s.get('kind')=='drive' and s.get('count',0) for _,s in related):
        add('REVIEW','drive','Gazebo 구동 ROS 명령 수신 미확인. 어댑터·브리지 검사 필요')
    if not findings:
        add('OBSERVED','interfaces','/clock·카메라·LiDAR·중앙 포즈와 명령 경로의 일부가 관측됨. 실제 랩 완주 미인증')
    status='FAIL' if any(x['status']=='FAIL' for x in findings) else 'REVIEW' if any(x['status']=='REVIEW' for x in findings) else 'OBSERVED'
    return {'status':status,'findings':findings}


def capture_live(root,team,track,seconds):
    script=root/'race/scripts/environment_diagnostic_v15.py'
    if not script.is_file():
        raise ValueError('v15 환경 진단기가 설치되어 있지 않습니다')
    outfile=(root/'race/results/diagnostics'/f'preflight_v16_{team}_{datetime.now().strftime("%Y%m%d_%H%M%S")}_{uuid.uuid4().hex[:5]}.json')
    cmd=[sys.executable,str(script),'capture','--root',str(root),'--team',team,
         '--track',track,'--seconds',str(seconds),'--label','competition','--output',str(outfile)]
    result=subprocess.run(cmd,check=False)
    if result.returncode or not outfile.is_file():
        raise RuntimeError(f'실시간 진단 실패: exit={result.returncode}')
    return summarize_live(json.loads(outfile.read_text(encoding='utf-8')),team),outfile


def render(reports, live=None):
    registered=[r for r in reports if r['status'] not in ('EMPTY','MISSING')]
    for r in registered:
        print(f"\n[차량 검수] {r['team']} : {r['status']}")
        for err in r['errors']:print('  FAIL 이유: '+err)
        for warn in r['warnings']:print('  확인 필요: '+warn)
        if not r['errors'] and not r['warnings']:
            print('  기본 파일·차량 구조 검사 통과')
        for stage,laps in [('예선',2),('본선 A/B',3),('결승',5)]:
            status='BLOCKED' if r['status']=='FAIL' else '실주행 미검증'
            print(f'  {stage} {laps}랩 준비: {status}')
    if not registered:
        print('[차량 검수] 등록 참가팀 없음. src/team/team_01~team_12에 제출물을 넣으십시오.')
    if live:
        print('\n[실시간 통신 진단] '+live['status'])
        for finding in live['findings']:
            print(f"  {finding['status']} {finding['area']}: {finding['detail']}")
    print('\n주의: 정적 검수와 통신 관측만으로 예선·본선·결승 완주를 보장하지 않습니다.')


def main(argv=None):
    parser=argparse.ArgumentParser(description='URRC v16 차량 정적/실시간 통신 검수')
    parser.add_argument('--root',type=Path,default=Path.home()/'urrc_f1_ws')
    parser.add_argument('--team',choices=TEAM_IDS)
    parser.add_argument('--live',action='store_true',help='실행 중 대회 Gazebo 데이터 수신 검사')
    parser.add_argument('--track',choices=TRACKS,default='monza')
    parser.add_argument('--seconds',type=float,default=20)
    parser.add_argument('--no-convert',action='store_true',help='Gazebo SDF 변환 검사 생략')
    parser.add_argument('--report',type=Path,help='JSON 검사 결과를 저장할 경로')
    args=parser.parse_args(argv)
    if args.live and not args.team:
        parser.error('--live에는 --team team_01 형식의 지정이 필요합니다')
    if not 3<=args.seconds<=600:
        parser.error('--seconds는 3~600초 사이여야 합니다')
    root=args.root.expanduser().resolve()
    if not (root/'src/team').is_dir():
        parser.error('src/team 폴더가 없습니다')
    # `--convert` is opt-in because it requires the installed Gazebo binaries.
    reports=check_entries(root,args.team,conversion=not args.no_convert)
    live=None;outfile=None
    if args.live and any(r['team']==args.team and r['status']!='FAIL' for r in reports):
        try:
            live,outfile=capture_live(root,args.team,args.track,args.seconds)
        except (RuntimeError,ValueError,OSError) as exc:
            live={'status':'FAIL','findings':[{'status':'FAIL','area':'diagnostic','detail':str(exc)}]}
    elif args.live:
        live={'status':'FAIL','findings':[{'status':'FAIL','area':'preflight','detail':'정적 검수 FAIL: 먼저 문제 수정 필요'}]}
    render(reports,live)
    data={'version':16,'created':datetime.now().astimezone().isoformat(),
          'registered':sum(r['status'] not in ('EMPTY','MISSING') for r in reports),
          'teams':reports,'live':live,'live_capture':str(outfile) if outfile else None}
    if args.report:
        args.report.parent.mkdir(parents=True,exist_ok=True)
        args.report.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print('[v16] 검수 보고서:',args.report)
    return 2 if not data['registered'] or any(r['status']=='FAIL' for r in reports) or (live and live['status']=='FAIL') else 0


if __name__=='__main__':
    sys.exit(main())
