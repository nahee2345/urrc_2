#!/usr/bin/env python3
"""URRC F1 operator-only 6-track compatibility audit; never edits entrant code.

Static audit: race assets, starting-signal consistency, entrant package/topic identity.
Live audit: reuses official v21/v22 camera-ready, GUI READY, signal and 2/3-lap
rehearsal pipeline; evidence is read from newly-created v17 nonofficial reports.
A two-track PASS never means four hidden tracks have passed.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

TRACKS=('monza','hungaroring','spa','cota','baku','monaco')
PRACTICE=('monza','hungaroring')
TEAM_IDS=tuple(f'team_{n:02d}' for n in range(1,13))
REQUIRED=('entry_id','ros_package','algorithm_launch_file','vehicle_model_file',
          'speed_topic','steering_topic','lidar_topic')
TOPIC_RE=re.compile(r'^/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*$')
IGNORE_COMMON={'/clock','/tf','/tf_static','/rosout','/parameter_events'}


def read_json(path:Path):
    with path.open(encoding='utf-8') as f:return json.load(f)


def save_json(path:Path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_name(path.name+'.'+uuid.uuid4().hex[:8]+'.tmp')
    try:
        temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        os.replace(temp,path)
    finally:temp.unlink(missing_ok=True)


def valid_track_selection(raw:str):
    if raw in ('all','6'):return TRACKS
    if raw in ('practice','2'):return PRACTICE
    tracks=tuple(dict.fromkeys(p.strip().lower() for p in raw.split(',') if p.strip()))
    if not tracks or any(t not in TRACKS for t in tracks):
        raise ValueError('맵은 all, practice 또는 monza,spa 형식의 6개 맵 이름만 허용됩니다')
    return tracks


def check_world(root:Path,track:str):
    """No user model touched. All checks are static, not claims of live readiness."""
    p=root/'src/urrc_track_gazebo'
    world=p/'worlds'/f'{track}_race.sdf'
    gui=p/'worlds'/f'{track}_grid.gui.config'
    active=p/'worlds'/f'{track}_signal_active.sdf'
    meta=p/'tracks/processed'/f'{track}.json'
    model=p/'models'/f'urrc_{track}/model.sdf'
    errors=[];warnings=[];checks={}
    for label,path in [('race_sdf',world),('gui',gui),('signal_active',active),
                       ('grid_json',meta),('track_model',model)]:
        if not path.is_file():errors.append(f'{label} 파일 없음: {path.relative_to(root)}')
    if world.is_file():
        try:
            top=ET.parse(world).getroot()
            w=top.find('world')
            if w is None or w.get('name')!=track:errors.append('월드 이름/구조 불일치')
            else:
                plugins={x.get('name','').split('::')[-1] for x in w.findall('plugin')}
                for plugin in ('Physics','UserCommands','SceneBroadcaster','Sensors'):
                    if plugin not in plugins:errors.append(f'필수 Gazebo 시스템 없음: {plugin}')
                cameras=[s for s in w.iter('sensor') if 'camera' in s.get('type','').lower()]
                if cameras:errors.append('맵 외부 카메라가 남아 있습니다(차량 장착 카메라만 허용)')
                if w.find("model[@name='urrc_v10_spectator']") is not None:
                    errors.append('관전용 카메라 모델이 남아 있습니다')
                gantry=w.find("model[@name='race_start_gantry']")
                if gantry is None:errors.append('5등 신호등 구조물 없음')
                else:
                    signatures=[]
                    for i in range(1,6):
                        v=gantry.find(f"link[@name='gantry_visuals']/visual[@name='red_lens_{i}']")
                        if v is None or v.findtext('geometry/box/size') is None:
                            errors.append(f'빨간 렌즈 {i} 누락')
                        else:
                            size=tuple(round(float(x),5) for x in v.findtext('geometry/box/size').split())
                            pos=tuple(round(float(x),5) for x in v.findtext('pose','').split())
                            signatures.append((size,pos))
                    if len(signatures)==5:checks['signal_signature']=signatures
                inc=[x.findtext('uri','') for x in w.findall('include')]
                if f'model://urrc_{track}' not in inc:
                    errors.append('대회 맵 모델 include 누락')
                ground=w.find("model[@name='ground']")
                if ground is None:errors.append('도로 외부 바닥 모델 없음')
                scene=w.find('scene')
                if scene is None:errors.append('Gazebo 배경 scene 누락')
                checks['world_file']=str(world.relative_to(root))
        except (ET.ParseError,ValueError,TypeError,OverflowError) as e:
            errors.append('월드 XML 검사 실패: '+str(e))
    if gui.is_file():
        src=gui.read_text(encoding='utf-8')
        # Gazebo GUI configs can be valid multiple-root XML fragments.
        src=re.sub(r'^\s*<\?xml[^>]*\?>','',src,count=1)
        try:
            g=ET.fromstring('<fragment>'+src+'</fragment>')
            plugins={x.get('filename') for x in g.findall('plugin')}
            if 'MinimalScene' not in plugins:errors.append('Gazebo 3D 렌더러 설정 누락')
            if 'CameraTracking' in plugins:errors.append('기존 차량 추적 GUI가 남아 있습니다')
        except ET.ParseError as e:errors.append('GUI 설정 XML 검사 실패: '+str(e))
    if active.is_file():
        try:
            a=ET.parse(active).getroot()
            mod=a.find('model')
            if mod is None or mod.get('name')!='urrc_start_signal_active':
                errors.append('단일 동시소등 신호등 모델 이름 불일치')
            else:
                lenses=[v.get('name') for v in mod.findall('link/visual')]
                if sorted(lenses)!=[f'active_red_lens_{n}' for n in range(1,6)]:
                    errors.append('단일 소등 모델의 활성 렌즈 5개 구성 불일치')
        except ET.ParseError as e:errors.append('신호 모델 XML 검사 실패: '+str(e))
    if meta.is_file():
        try:
            data=read_json(meta)
            slots=data.get('grid',[])
            if not isinstance(slots,list) or len(slots)<12:
                errors.append('출발 그리드 12칸 미만 또는 형식 오류')
            else:
                for index,slot in enumerate(slots[:12]):
                    if not isinstance(slot,dict) or not all(k in slot for k in ('x','y','yaw','road_z')):
                        errors.append(f'출발 그리드 #{index+1} 좌표 누락')
                        break
                checks['grid_slots']=len(slots)
        except (ValueError,TypeError,OSError) as e:errors.append('그리드 JSON 검사 실패: '+str(e))
    if model.is_file():
        try:
            doc=ET.parse(model).getroot()
            for required in ('road_collision','wall_outer_collision','wall_inner_collision'):
                if doc.find(f".//collision[@name='{required}']") is None:
                    errors.append('충돌체 누락: '+required)
        except ET.ParseError as e:errors.append('트랙 모델 XML 검사 실패: '+str(e))
    return {'track':track,'status':'FAIL' if errors else 'STATIC_PASS',
            'errors':errors,'warnings':warnings,'checks':checks}


def check_track_group(root:Path,tracks):
    result={track:check_world(root,track) for track in tracks}
    # Check signal geometry parity with Monza without assuming equal support widths.
    reference=check_world(root,'monza')['checks'].get('signal_signature')
    if reference:
        for t,row in result.items():
            signature=row['checks'].get('signal_signature')
            if signature and signature!=reference:
                row['errors'].append('Monza 기준 빨간 렌즈 5개 위치/크기와 불일치')
                row['status']='FAIL'
    return result


def submitted(root:Path):
    src=root/'src/team'
    official=root/'race/entrants'
    out={}
    for team in TEAM_IDS:
        folder=src/team
        origin='src/team'
        if not (folder/'race_entry.json').is_file():
            folder=official/team;origin='race/entrants'
        if (folder/'race_entry.json').is_file():
            out[team]=(folder,origin)
    return out


def read_team_entries(root:Path,strict_sync:bool=False):
    found=submitted(root)
    teams={};global_errors=[]
    if not found:global_errors.append('등록된 참가팀 없음: src/team/team_XX/race_entry.json을 접수하십시오')
    for team,(folder,origin) in found.items():
        errors=[];warnings=[];checks={'origin':origin}
        try:
            e=read_json(folder/'race_entry.json')
            if not isinstance(e,dict):raise ValueError('JSON 객체가 아닙니다')
            for key in REQUIRED:
                if not isinstance(e.get(key),str) or not e[key].strip():
                    errors.append('필수 설정 누락: '+key)
            if e.get('entry_id')!=team:errors.append('폴더명과 entry_id 불일치')
            mdl=e.get('vehicle_model_file')
            if isinstance(mdl,str) and mdl:
                p=(folder/mdl).resolve()
                if not p.is_relative_to(folder.resolve()) or not p.is_file():
                    errors.append('차량 모델 경로가 참가 폴더 내부의 실제 파일이 아닙니다')
            elif not mdl:errors.append('차량 모델 지정 없음')
            packages=[]
            for manifest in folder.rglob('package.xml'):
                if any(part in ('build','install','log','.git') for part in manifest.relative_to(folder).parts):continue
                try:
                    n=ET.parse(manifest).getroot().findtext('name')
                    if n:packages.append(n)
                except ET.ParseError:errors.append('ROS package.xml 파싱 실패: '+str(manifest.relative_to(folder)))
            if not packages:errors.append('ROS package.xml 없음')
            if e.get('ros_package') not in packages:errors.append('ros_package가 ROS package.xml과 일치하지 않습니다')
            checks['ros_packages']=sorted(set(packages))
            launch=e.get('algorithm_launch_file')
            if isinstance(launch,str) and launch and not any((p.parent.name=='launch' and p.name==launch) for p in folder.rglob(launch)):
                errors.append('launch 파일 없음: '+launch)
            off=root/'race/entrants'/team/'race_entry.json'
            if off.is_file():
                other=read_json(off)
                if not isinstance(other,dict) or any(e.get(k)!=other.get(k) for k in REQUIRED):
                    errors.append('공식 접수본과 원본 설정 불일치: sync 필요')
                # Compare exact submitted model bytes when layouts permit.
                elif isinstance(mdl,str):
                    op=root/'race/entrants'/team/mdl
                    if op.is_file() and (folder/mdl).is_file():
                        if hashlib.sha256(op.read_bytes()).digest()!=hashlib.sha256((folder/mdl).read_bytes()).digest():
                            errors.append('공식 접수본과 원본 차량 모델이 다릅니다: sync 필요')
            elif strict_sync:
                errors.append('공식 접수본 없음: bash race_v23.sh sync 필요')
            else:
                warnings.append('공식 접수본 없음: 대회 전에 sync → prepare 필요')
            teams[team]={'team':team,'entry':e,'status':'FAIL' if errors else 'STATIC_PASS',
                         'errors':errors,'warnings':warnings,'checks':checks}
        except (ValueError,TypeError,OSError) as e:
            teams[team]={'team':team,'entry':{},'status':'FAIL',
                         'errors':['race_entry.json 읽기 실패: '+str(e)],'warnings':[],'checks':checks}
    return teams,global_errors


def topic_fields(entry):
    """All explicitly declared ROS topics, not only the three legacy required fields."""
    for name,value in entry.items():
        if name.endswith('_topic') and isinstance(value,str) and value.strip():
            if name=='race_start_topic':
                # This is injected by the official race start gate per team.
                continue
            yield name,value.strip()


def check_topic_uniqueness(root:Path,teams:dict):
    used={};pkgs={};errors=[]
    mappings={}
    cfg=root/'race/camera_topics_v21.json'
    if cfg.is_file():
        try:
            value=read_json(cfg)
            if not isinstance(value,dict):raise ValueError('JSON 객체가 아닙니다')
            mappings=value
        except (ValueError,OSError) as e:errors.append('운영자 카메라 매핑 오류: '+str(e))
    for team,row in teams.items():
        e=row['entry']
        name=e.get('ros_package')
        if isinstance(name,str) and name:pkgs.setdefault(name,[]).append(team)
        topics=dict(topic_fields(e))
        if team in mappings:topics['operator_camera_topic']=mappings[team]
        for key,value in topics.items():
            if not isinstance(value,str) or not TOPIC_RE.fullmatch(value):
                row['errors'].append(f'{key}: ROS 절대 토픽 형식 오류: {value!r}')
                continue
            if value in IGNORE_COMMON:
                if key in ('speed_topic','steering_topic','lidar_topic'):
                    row['errors'].append(f'{key}: 공용 ROS 토픽을 차량 전용으로 지정할 수 없습니다: {value}')
                continue
            used.setdefault(value,[]).append((team,key))
        if 'camera_topic' not in topics and 'rgb_topic' not in topics and 'image_topic' not in topics and 'operator_camera_topic' not in topics:
            row['warnings'].append('카메라 토픽 지정 없음: v21 자동 탐색으로 확인, 모호하면 race/camera_topics_v21.json 설정 필요')
        elif 'operator_camera_topic' not in topics:
            n=team[5:]
            declared=[t.lower() for k,t in topics.items() if k in ('camera_topic','rgb_topic','image_topic','start_camera_topic')]
            if declared and not any(re.search(rf'(^|/)(team{n}|team_{n})(/|$)|_{n}(/|$)',t) for t in declared):
                row['warnings'].append('카메라 토픽 이름으로 v21 팀 자동 식별이 어려움: 운영자 camera_topics_v21.json에 등록 필요')
    for name,refs in pkgs.items():
        if len(set(refs))>1:
            for team in set(refs):teams[team]['errors'].append(f'다른 팀과 ROS 패키지명 중복: {name}, {sorted(set(refs))}')
    for topic,refs in used.items():
        group={team for team,_ in refs}
        if len(group)>1:
            for team in group:
                teams[team]['errors'].append(f'다른 팀과 토픽 중복: {topic} ({", ".join(f"{t}.{k}" for t,k in refs)})')
    for team,row in teams.items():
        row['errors']=sorted(set(row['errors']))
        row['warnings']=sorted(set(row['warnings']))
        row['status']='FAIL' if row['errors'] else 'STATIC_PASS'
    return errors


def inspect_additional_v16(root:Path,teams:dict):
    sys.path.insert(0,str(root/'race/scripts'))
    try:
        from vehicle_preflight_v16 import check_entries
        reports=check_entries(root,conversion=False)
    except ImportError:
        return ['v16 정적 차량 검수기를 찾지 못했습니다: 설치 구성 확인 필요']
    except (OSError,ValueError,TypeError,ET.ParseError) as e:
        return ['v16 정적 차량 검사 실행 오류: '+str(e)]
    for report in reports:
        row=teams.get(report.get('team'))
        if row is None:continue
        row['checks']['v16_status']=report['status']
        row['errors'].extend(report.get('errors',[]))
        row['warnings'].extend(report.get('warnings',[]))
        if row['errors']:row['status']='FAIL'
    return []


def check_installation(root:Path):
    errors=[]
    for f in ('race_v22.sh','race_v21.sh','race/scripts/stage_preview_v17.py',
              'race/scripts/race_camera_ready_v21.py','race/scripts/vehicle_preflight_v16.py'):
        if not (root/f).is_file():errors.append('운영 코드 없음: '+f)
    return errors


def check_live_result(root:Path,team:str,track:str,before:set[Path],code:int):
    directory=root/'race/previews'
    after={p for p in directory.glob('v17_*') if p.is_dir()} if directory.is_dir() else set()
    fresh=sorted(after-before,key=lambda p:p.stat().st_mtime,reverse=True)
    failures=[]
    if code:failures.append(f'모의주행 종료코드 {code}')
    if len(fresh)!=1:failures.append('모의주행 생성 결과가 정확히 1개가 아닙니다')
    if not fresh:
        return {'track':track,'status':'FAIL','errors':failures,'report':None}
    file=fresh[0]/'report.json'
    if not file.is_file():
        failures.append('모의주행 report.json 미생성')
        return {'track':track,'status':'FAIL','errors':failures,'report':str(file)}
    try:
        data=read_json(file)
        if data.get('track')!=track or team not in data.get('teams',[]):failures.append('대상 트랙/팀 결과가 일치하지 않습니다')
        if data.get('status')!='OBSERVATION_COMPLETE':failures.append('실주행 완료 상태가 아닙니다: '+str(data.get('status')))
        result=data.get('details',{}).get(team,{})
        if result.get('status')!='PASS' or result.get('finish_time') is None:
            failures.append('실제 2/3랩 완주 미확인: '+str(result.get('reason','INCOMPLETE')))
        if data.get('sensor_status')!='CAPTURED' or data.get('sensor_world_match') is not True:
            failures.append('맵 일치·카메라/LiDAR 센서 진단 증거 부족')
        findings=data.get('sensor_summary',{}).get('teams',{}).get(team,[])
        if not isinstance(findings,list):
            failures.append('팀별 센서 진단 목록 없음')
        else:
            status_by_area={f.get('area'):f.get('status') for f in findings if isinstance(f,dict)}
            # v15 sensor diagnostic classifies missing camera/LiDAR as UNKNOWN,
            # which must NOT be treated as a pass for race readiness.
            for area in ('camera','lidar','timing_pose','speed_steering'):
                if status_by_area.get(area)!='PASS':
                    failures.append(f'{area} 실제 ROS 데이터 PASS 미확인: {status_by_area.get(area, "MISSING")}')
            if any(v in ('FAIL','UNKNOWN') for v in status_by_area.values()):
                failures.append('센서 진단 FAIL/UNKNOWN 포함')
        # v21 pre-light camera-ready evidence proves 2 actual frames + /clock.
        frame_files=list(fresh[0].glob('camera_ready_v21_*.json'))
        if len(frame_files)!=1:
            failures.append('v21 출발 전 카메라 프레임/clock 검사 결과가 정확히 1개가 아닙니다')
        else:
            ready=read_json(frame_files[0])
            if (ready.get('complete') is not True or
                    ready.get('frames',{}).get(team,0)<2 or
                    ready.get('clock_progress_samples',0)<2):
                failures.append('신호등 작동 전 실제 카메라 프레임 2개 및 /clock 진행 미확인')
            if not TOPIC_RE.fullmatch(str(ready.get('topics',{}).get(team,''))):
                failures.append('v21에서 확인한 카메라 토픽 이름이 누락되거나 잘못됐습니다')
        if result.get('max_displacement_m') is not None and float(result['max_displacement_m'])<0.5:
            failures.append('차량 실제 이동량 0.5 m 미만')
        if result.get('speed_messages_after_release') is not None and int(result['speed_messages_after_release'])<1:
            failures.append('출발 이후 속도 제어 명령 없음')
    except (OSError,ValueError,TypeError) as e:failures.append('실주행 결과 읽기 실패: '+str(e))
    return {'track':track,'status':'FAIL' if failures else 'LIVE_PASS',
            'errors':failures,'report':str(file)}


def live_per_track(root:Path,track:str,ids:list[str],seconds:int):
    before={p for p in (root/'race/previews').glob('v17_*') if p.is_dir()} if (root/'race/previews').is_dir() else set()
    stage='qualifying' if len(ids)==1 else 'semi_a'
    cmd=['bash',str(root/'race_v22.sh'),'rehearse',stage,
         '--track',track,'--seconds',str(seconds)]
    if len(ids)==1:cmd+=['--team',ids[0]]
    else:cmd+=['--teams',','.join(ids)]
    print(f'[v23 LIVE] {track} / {", ".join(ids)} / 운영자 READY 필요',flush=True)
    code=subprocess.call(cmd,cwd=root)
    findings={team:check_live_result(root,team,track,before,code) for team in ids}
    # The controller readiness probe sees real, per-car image topics; a single
    # shared ROS camera topic cannot represent two distinct car cameras.
    if len(ids)>1:
        after={p for p in (root/'race/previews').glob('v17_*') if p.is_dir()}
        fresh=after-before
        if len(fresh)==1:
            ready_files=list(next(iter(fresh)).glob('camera_ready_v21_*.json'))
            if len(ready_files)==1:
                try:
                    topics=read_json(ready_files[0]).get('topics',{})
                    values=[topics.get(team) for team in ids]
                    if len(values)!=len(set(values)):
                        for team in ids:
                            findings[team]['errors'].append('실제 ROS 카메라 영상 토픽이 다른 팀과 중복됩니다')
                            findings[team]['status']='FAIL'
                except (OSError,ValueError):pass
    return findings


def write_reports(root:Path,selection,teams:dict,maps:dict,errors:list,live:dict|None):
    out=root/'race/results/compatibility_v23'
    out.mkdir(parents=True,exist_ok=True)
    time_iso=datetime.now(timezone.utc).astimezone().isoformat()
    # Never mutate runtime entrant metadata when writing a public report.
    redacted={team:{**row,'entry':{k:v for k,v in row['entry'].items()
                                   if k in REQUIRED or k.endswith('_topic')}}
              for team,row in teams.items()}
    result={'version':23,'timestamp':time_iso,'tracks':list(selection),
            'maps':maps,'teams':redacted,'errors':errors,'live':live or {}}
    save_json(out/'summary.json',result)
    lines=['URRC F1 v23 호환성 검사', '검사일시: '+time_iso,
           '대상맵: '+', '.join(selection),'']
    for tr,entry in maps.items():
        lines.append('MAP '+tr+' '+entry['status']+(' : '+'; '.join(entry['errors']) if entry['errors'] else ''))
    for team,row in teams.items():
        live_status=' | '.join(f'{t}:{d["status"]}' for t,d in (live or {}).get(team,{}).items())
        line=team+' '+row['status']+(' | '+live_status if live_status else '')
        lines.append(line)
        full=[line]+['  '+kind+': '+x for kind,items in (('FAIL',row['errors']),('확인',row['warnings'])) for x in items]
        if live and team in live:
            full+=['  '+t+' '+v['status']+' '+('; '.join(v['errors'])) for t,v in live[team].items()]
        (out/(team+'.txt')).write_text('\n'.join(full)+'\n',encoding='utf-8')
    lines.extend(['']+['FAIL: '+e for e in errors])
    lines+=['', 'STATIC_PASS는 6개 맵 실주행·완주 보장이 아닙니다.',
            'LIVE_PASS는 해당 맵·해당 주행 조건에서만 유효합니다.']
    (out/'summary.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('[v23] 검사 보고서: '+str(out/'summary.txt'),flush=True)
    return result


def main(argv=None):
    parser=argparse.ArgumentParser(description='URRC F1 v23 팀 토픽·6맵 호환성 검사')
    parser.add_argument('--root',type=Path,default=Path.home()/'urrc_f1_ws')
    parser.add_argument('--team',choices=TEAM_IDS,help='단일팀 지정')
    parser.add_argument('--teams',help='복수팀 동시주행 대상 team_01,team_02')
    parser.add_argument('--tracks',default='all',help='all, practice, monza,spa,...')
    parser.add_argument('--live',action='store_true',help='선택된 모든 맵에서 실제 모의주행; GUI READY 필요')
    parser.add_argument('--seconds',type=int,default=300)
    parser.add_argument('--require-synced',action='store_true',help='공식 접수본 없는 경우 FAIL')
    parser.add_argument('--prepare-worlds',action='store_true',help='연습·대회 월드 표면 리소스 사전 생성')
    parser.add_argument('--worlds-only',action='store_true',help='참가팀 없이 6개 맵 자체만 검사')
    a=parser.parse_args(argv)
    root=a.root.expanduser().resolve()
    if a.team and a.teams:parser.error('--team과 --teams를 함께 지정할 수 없습니다')
    if not 15<=a.seconds<=2400:parser.error('--seconds는 15~2400초 범위입니다')
    try:
        tracks=valid_track_selection(a.tracks)
        if a.teams:
            ids=list(dict.fromkeys(x.strip() for x in a.teams.split(',') if x.strip()))
            if not ids or any(t not in TEAM_IDS for t in ids):raise ValueError('--teams는 team_01~team_12만 지정하십시오')
            if len(ids)>6:raise ValueError('동시 모의주행 참가자는 최대 6팀입니다')
        else:ids=[a.team] if a.team else []
        if a.live and not ids:
            raise ValueError('실주행 검사는 --team 또는 --teams를 지정하십시오')
        if a.worlds_only and (a.live or ids or a.require_synced):
            raise ValueError('--worlds-only는 팀 지정, 실주행, 공식 접수 검사와 함께 사용할 수 없습니다')
        errors=check_installation(root)
        if a.prepare_worlds:
            sys.path.insert(0,str(root/'src/urrc_track_gazebo'))
            from urrc_tracks.v10_assets import enhance_track
            if 'v22 spectator cameras disabled' not in (root/'src/urrc_track_gazebo/urrc_tracks/v10_assets.py').read_text(encoding='utf-8'):
                raise ValueError('v22 관전 카메라 제거 코드가 없어 맵 리소스 갱신을 차단합니다')
            for t in tracks:
                print('[v23] 맵 시각 리소스 준비: '+t,flush=True)
                enhance_track(root/'src/urrc_track_gazebo',t)
        maps=check_track_group(root,tracks)
        teams,discover_errors=({},[]) if a.worlds_only else read_team_entries(root,strict_sync=(a.require_synced or a.live))
        errors+=discover_errors
        if not a.worlds_only:
            errors+=check_topic_uniqueness(root,teams)
            errors+=inspect_additional_v16(root,teams)
        if a.team:
            if a.team not in teams:errors.append('선택 참가팀 미접수: '+a.team)
        if ids and any(team not in teams for team in ids):errors.append('일부 실주행 참가팀 미접수')
        fail=bool(errors or any(r['status']=='FAIL' for r in maps.values()) or any(r['status']=='FAIL' for r in teams.values()))
        live_results={}
        if a.live:
            if fail:
                errors.append('정적 검사 FAIL이므로 차량 스폰·신호등 작동 없이 실주행 차단')
            else:
                if not (root/'race/install/setup.bash').is_file():
                    errors.append('ROS 빌드 없음: bash race_v23.sh prepare 먼저 실행하십시오')
                    fail=True
                else:
                    for t in tracks:
                        results=live_per_track(root,t,ids,a.seconds)
                        for team,data in results.items():live_results.setdefault(team,{})[t]=data
                        if any(data['status']!='LIVE_PASS' for data in results.values()):
                            fail=True
                        write_reports(root,tracks,teams,maps,errors,live_results)
        write_reports(root,tracks,teams,maps,errors,live_results)
        for t,row in maps.items():print('[v23 MAP]',t,row['status'],('; '.join(row['errors'])) if row['errors'] else '',flush=True)
        for t,row in teams.items():print('[v23 TEAM]',t,row['status'],('; '.join(row['errors'])) if row['errors'] else '',flush=True)
        for err in errors:print('[v23 FAIL]',err,file=sys.stderr,flush=True)
        if live_results:
            for t,rows in live_results.items():
                for track,entry in rows.items():print('[v23 LIVE]',t,track,entry['status'],flush=True)
        return 2 if fail or errors else 0
    except (ValueError,OSError,ET.ParseError,ImportError) as exc:
        print('[v23] 호환성 검사 실행 실패: '+str(exc),file=sys.stderr,flush=True)
        return 2
    except KeyboardInterrupt:
        print('[v23] 운영자 중단: 미완료 테스트는 PASS 처리하지 않습니다',file=sys.stderr)
        return 130

if __name__=='__main__':sys.exit(main())
