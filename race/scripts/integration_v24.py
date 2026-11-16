#!/usr/bin/env python3
"""Per-folder static, Gazebo-world, ROS /clock and optional GUI/vehicle integration checks.

Every PASS requires actual evidence. No entrant => no car/camera/lap assertions.
"""
from __future__ import annotations
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


def command(args, *, env=None, timeout=30, cwd=None):
    return subprocess.run(args, env=env, cwd=cwd, capture_output=True, text=True,
                          timeout=timeout, check=False)


def stop(proc):
    if proc is None or proc.poll() is not None:
        return
    try:os.killpg(proc.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        try:os.killpg(proc.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            proc.wait(timeout=3)


def profile(root):
    data=json.loads((root/'race/v24_profile.json').read_text(encoding='utf-8'))
    if data.get('mode') not in ('practice','official'):
        raise ValueError('v24 프로필 오류')
    wanted=['monza','hungaroring','monaco'] if data['mode']=='practice' else ['monza','hungaroring','monaco','spa','cota','baku']
    if data.get('tracks')!=wanted:
        raise ValueError('지원 맵 목록이 v24 프로필과 다릅니다')
    folder='urrc_2' if data['mode']=='practice' else 'urrc_22'
    if root.name!=folder:
        raise ValueError(f'작업 폴더 이름은 {folder}이어야 합니다: {root}')
    return data['mode'],wanted


def check_static(root, tracks,log_dir):
    checks=[['python3',str(root/'race/scripts/verify_v24.py'),str(root)],
            ['bash',str(root/'race_v24.sh'),'compatibility','--worlds-only','--tracks','all']]
    results=[]
    for i,cmd in enumerate(checks,1):
        ret=command(cmd,cwd=root,timeout=130)
        (log_dir/f'static_{i}.log').write_text(ret.stdout+'\n'+ret.stderr,encoding='utf-8')
        results.append({'command':' '.join(cmd), 'status':'PASS' if ret.returncode==0 else 'FAIL',
                        'reason':(ret.stderr or ret.stdout)[-1500:] if ret.returncode else ''})
        print(f'[v24 통합] 정적 검사 {i}/2:',results[-1]['status'],flush=True)
        if ret.returncode:break
    return results


def world_headless(root, track, log_dir, ready_seconds=90):
    pkg=root/'src/urrc_track_gazebo'
    path=pkg/'worlds'/f'{track}_race.sdf'
    env=os.environ.copy()
    env['GZ_PARTITION']=f'urrc_v24_verify_{os.getpid()}_{track}'
    env['GZ_SIM_RESOURCE_PATH']=os.pathsep.join(filter(None,[str(pkg/'models'),env.get('GZ_SIM_RESOURCE_PATH','')]))
    # Use a separate ROS domain for each verification run to avoid a false
    # /clock PASS from another Gazebo instance on the user's PC.
    env['ROS_DOMAIN_ID']=str(180+(os.getpid()%40))
    gz_log=log_dir/f'{track}_gazebo.log'
    bridge_log=log_dir/f'{track}_bridge.log'
    result={'track':track,'status':'FAIL','gazebo_log':str(gz_log),'bridge_log':str(bridge_log),'checks':[],'errors':[]}
    server=None;bridge=None
    try:
        with gz_log.open('w',encoding='utf-8') as gs, bridge_log.open('w',encoding='utf-8') as bs:
            server=subprocess.Popen(['gz','sim','-s','-r',str(path)],stdout=gs,stderr=subprocess.STDOUT,
                                    env=env,cwd=root,start_new_session=True)
            deadline=time.monotonic()+ready_seconds
            required={f'/world/{track}/control',f'/world/{track}/create',f'/world/{track}/remove',f'/world/{track}/visual_config'}
            observed=set()
            while time.monotonic()<deadline:
                if server.poll() is not None:
                    raise RuntimeError(f'Gazebo 서버 조기 종료: exit={server.returncode}')
                out=command(['gz','service','-l'],env=env,timeout=10)
                if out.returncode==0:
                    observed=set(out.stdout.split())
                    if required.issubset(observed):break
                time.sleep(1.4)
            else:
                raise TimeoutError('Gazebo 월드 핵심 서비스 준비 시간 초과: '+', '.join(sorted(required-observed)))
            result['checks'].append('Gazebo server services: control/create/remove/visual_config')
            # Bridge is started AFTER the world; it must receive live /clock.
            bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
                                     '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
                                    env=env,cwd=root,stdout=bs,stderr=subprocess.STDOUT,start_new_session=True)
            times=[]
            for i in range(2):
                try:
                    sample=command(['ros2','topic','echo','/clock','--once'],env=env,cwd=root,timeout=25)
                except subprocess.TimeoutExpired as e:
                    raise TimeoutError('ROS /clock 수신 시간 초과') from e
                if sample.returncode:
                    raise RuntimeError('ROS /clock 프레임 수신 실패: '+(sample.stderr or sample.stdout)[-500:])
                found=re.search(r'\bsec:\s*(\d+)\s+nanosec:\s*(\d+)',sample.stdout)
                if not found:
                    raise RuntimeError('ROS /clock 메시지 형식 확인 실패: '+sample.stdout[-500:])
                times.append(int(found[1])*1_000_000_000+int(found[2]))
                if i==0:time.sleep(.4)
            if times[1]<=times[0]:
                raise RuntimeError('ROS /clock이 진행되지 않습니다')
            result['checks'].append('ROS bridge live /clock 2 progressing frames')
            result['status']='RUNTIME_WORLD_PASS'
    except (OSError,RuntimeError,TimeoutError,subprocess.SubprocessError) as e:
        result['errors'].append(str(e))
    finally:
        stop(bridge)
        stop(server)
    return result


def gui_check(root,track):
    print(f'\n[v24 GUI] {track}: 화면이 로딩되고 도로·그리드·신호등이 보이면 READY를 입력하십시오.',flush=True)
    print('[v24 GUI] 신호 5개 점등·동시 소등까지 확인한 다음 Gazebo 창을 닫으십시오.',flush=True)
    proc=None
    try:
        proc=subprocess.Popen(['bash',str(root/'race_v24.sh'),'preview','qualifying','--track',track],cwd=root)
        try:proc.wait()
        except KeyboardInterrupt:
            print('\n[v24 GUI] 운영자 종료 요청',flush=True)
            # Preview should handle its own SIGINT as it shares foreground group.
            try:proc.wait(timeout=4)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:proc.kill();proc.wait()
        answer=input(f'[v24 GUI] {track}의 로딩·신호등 표시와 점등/소등을 실제 확인하셨습니까? [y/N] ').strip().lower()
        return {'track':track,'status':'GUI_OPERATOR_PASS' if answer=='y' else 'FAIL',
                'errors':[] if answer=='y' else ['운영자 GUI 확인 미완료 또는 실패']}
    except (KeyboardInterrupt,EOFError):
        return {'track':track,'status':'FAIL','errors':['GUI 확인 취소']}
    finally:
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()


def write_summary(root,mode,checks,worlds,team_check=None):
    out=root/'race/results/integration_v24'
    out.mkdir(parents=True,exist_ok=True)
    ok=bool(checks) and all(x['status']=='PASS' for x in checks) and all(x['status'].endswith('_PASS') for x in worlds)
    if team_check is not None:ok=ok and team_check.get('status')=='LIVE_PASS'
    payload={'timestamp':datetime.now(timezone.utc).astimezone().isoformat(),
             'workspace':str(root),'profile':mode,'status':'PASS' if ok else 'FAIL',
             'static_checks':checks,'maps':worlds,
             'team_test':team_check or {'status':'NOT_RUN','reason':'참가팀이 없는 경우 차량·카메라·완주는 검증하지 않음'}}
    (out/'summary.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    lines=[f'URRC F1 v24 통합 검증: {root.name} ({mode})',f'검사 시각: {payload["timestamp"]}',
           f'결과: {payload["status"]}',
           *[f'정적 검사: {x["status"]} {x["reason"]}' for x in checks],
           *[f'맵 {x["track"]}: {x["status"]} {"; ".join(x["errors"])}' for x in worlds],
           '차량/센서/주행: '+payload['team_test']['status']]
    (out/'summary.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('\n'.join(lines),flush=True)
    print('결과 파일:',out/'summary.txt',flush=True)
    return 0 if ok else 2


def main(argv=None):
    p=argparse.ArgumentParser(description='v24 folder-level integrated checks: static + actual Gazebo/ROS')
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    p.add_argument('--gui',action='store_true',help='operator confirms each GUI scene/light sequence')
    p.add_argument('--team',help='team_01~team_12: run camera+LiDAR+finish test after maps')
    p.add_argument('--seconds',type=int,default=300)
    p.add_argument('--timeout',type=int,default=90,help='max seconds for each Gazebo world readiness')
    a=p.parse_args(argv)
    root=a.root.expanduser().resolve()
    try:mode,tracks=profile(root)
    except (OSError,ValueError,json.JSONDecodeError) as e:
        print('[v24 통합 FAIL]',e,file=sys.stderr)
        return 2
    if a.team and not re.fullmatch(r'team_(?:0[1-9]|1[0-2])',a.team):
        p.error('--team은 team_01~team_12만 가능합니다')
    if not (15<=a.seconds<=2400):p.error('--seconds 15..2400')
    if not 15<=a.timeout<=300:p.error('--timeout 15..300')
    log=root/'race/results/integration_v24'
    log.mkdir(parents=True,exist_ok=True)
    checks=check_static(root,tracks,log)
    if len(checks)!=2 or any(x['status']!='PASS' for x in checks):
        return write_summary(root,mode,checks,[])
    if not all(shutil.which(n) for n in ('gz','ros2')) and not a.gui:
        fail={'track':'ALL','status':'NOT_RUN','errors':['Gazebo 또는 ROS2 명령을 찾지 못했습니다']}
        return write_summary(root,mode,checks,[fail])
    worlds=[]
    for name in tracks:
        print(f'[v24 통합] {name}: {"GUI 수동 검사" if a.gui else "Gazebo + ROS 브리지 실구동 검사"}',flush=True)
        entry=gui_check(root,name) if a.gui else world_headless(root,name,log,a.timeout)
        worlds.append(entry)
        print('[v24 통합]',name,entry['status'],flush=True)
    live=None
    if a.team:
        cmd=['bash',str(root/'race_v24.sh'),'compatibility','--team',a.team,
             '--live','--tracks','all','--seconds',str(a.seconds)]
        print('[v24 통합] 실제 차량·센서·완주 검증:',a.team,flush=True)
        code=subprocess.call(cmd,cwd=root)
        live={'status':'LIVE_PASS' if code==0 else 'FAIL', 'exit_code':code}
    return write_summary(root,mode,checks,worlds,live)

if __name__=='__main__':sys.exit(main())
