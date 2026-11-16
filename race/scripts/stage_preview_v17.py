#!/usr/bin/env python3
"""Operator-only preview and race-like rehearsal; never write official results.

`preview`: inspect a track in a Gazebo GUI, without cars or official scoring.
`rehearse`: use the same race_manager world, model, controller, gate and
lap monitor as the actual race; store *only* under race/previews/v17_*/.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid

STAGES=('vehicles','qualifying','semi_a','semi_b','final')
TRACKS=('monza','spa','monaco','cota','hungaroring','baku')
TEAM_RE=re.compile(r'team_(?:0[1-9]|1[0-2])\Z')
LAPS={'vehicles':2,'qualifying':2,'semi_a':3,'semi_b':3,'final':5}
# Defaults only for first-time previews without an official map assignment.
MAP_FALLBACK={'vehicles':'monza','qualifying':'spa','semi_a':'cota',
              'semi_b':'baku','final':'monaco'}


def choose_track(root:Path, stage:str, explicit:str|None)->tuple[str,str]:
    if explicit:
        if explicit not in TRACKS:
            raise ValueError('지원하지 않는 맵: '+explicit)
        return explicit,'명시적으로 지정한 맵 (공식 배정은 변경되지 않음)'
    if stage=='vehicles':
        return 'monza','차량검수 기본 맵'
    assignment=root/'race/results/v3_maps.json'
    if assignment.is_file():
        data=json.loads(assignment.read_text(encoding='utf-8'))
        mapping=data.get('maps')
        if not isinstance(mapping,dict) or set(mapping)!={'qualifying','semi_a','semi_b','final'}:
            raise ValueError('대회 맵 배정 파일이 올바르지 않습니다: '+str(assignment))
        if len(set(mapping.values()))!=4 or any(x not in TRACKS for x in mapping.values()):
            raise ValueError('대회 맵 배정이 중복 또는 미지원 상태입니다')
        if mapping['qualifying']=='monaco' or mapping['final']=='monza':
            raise ValueError('대회 맵 배정 제한 위반')
        return mapping[stage],'기존 대회 단계별 맵 배정 (읽기 전용)'
    return MAP_FALLBACK[stage],'맵 배정 없음: 임시 기본 맵 (공식 배정 아님)'


def validate_team_names(ids:list[str])->list[str]:
    if not ids:
        raise ValueError('모의주행은 --team team_01 또는 --teams team_01,team_02 가 필요합니다')
    if any(not TEAM_RE.fullmatch(x) for x in ids):
        raise ValueError('팀 ID는 team_01~team_12 형식이어야 합니다: '+repr(ids))
    if len(set(ids))!=len(ids):
        raise ValueError('중복된 참가팀은 동시에 지정할 수 없습니다')
    return ids


def selected_entries(entries:list[dict], names:list[str],stage:str)->list[dict]:
    validate_team_names(names)
    limit=1 if stage in ('vehicles','qualifying') else (6 if stage!='final' else 7)
    if len(names)>limit:
        raise ValueError(f'{stage}: 모의주행에서 최대 {limit}대만 선택할 수 있습니다')
    byid={entry['entry_id']:entry for entry in entries}
    missing=[name for name in names if name not in byid]
    if missing:
        raise ValueError('공식 접수본이 없습니다: '+', '.join(missing)+'; 먼저 check/sync/prepare 진행')
    return [byid[name] for name in names]


def preflight_selected(root:Path,names:list[str])->None:
    from vehicle_preflight_v16 import check_entries
    from pathlib import Path
    for name in names:
        report=check_entries(Path(root),name,conversion=False)[0]
        if report['status'] in ('FAIL','EMPTY','MISSING'):
            raise ValueError(name+' 차량 정적 검수 '+report['status']+': '+
                             '; '.join(report.get('errors') or ['제출물 없음']))
        for message in report.get('warnings',[]):
            print('[v17 검수 REVIEW] '+name+': '+message,flush=True)


def preview_map(root:Path,stage:str,track:str)->int:
    # v19 Monza-equivalent preview + preview-only signal demo
    from preview_visual_v19 import ensure_assets,preview_signal_demo,stop_preview
    world=root/'src/urrc_track_gazebo/worlds'/f'{track}_race.sdf'
    gui=root/'src/urrc_track_gazebo/worlds'/f'{track}_grid.gui.config'
    if not world.is_file():
        raise ValueError('Gazebo 대회 월드 파일 없음: '+str(world))
    if not gui.is_file():
        raise ValueError('그리드 GUI 설정 없음: '+str(gui))
    if not (root/'src/urrc_track_gazebo/models').is_dir():
        raise ValueError('대회 맵 리소스 폴더가 없습니다')
    ensure_assets(root,track)
    env=os.environ.copy()
    env['GZ_PARTITION']='urrc_v19_preview_'+uuid.uuid4().hex[:10]
    old=env.get('GZ_SIM_RESOURCE_PATH','')
    env['GZ_SIM_RESOURCE_PATH']=os.pathsep.join(filter(None,[
        str(root/'src/urrc_track_gazebo/models'),old]))
    print('[v19 MAP] '+stage+' / '+track+' / 차량 없음 / 공식 기록 변경 없음',flush=True)
    # v21 preview with no vehicle: 2-4 and 6 cannot be tested without entrants.
    print('[v21 START] 1/6 맵 실행 → 2~4/6 차량·센서 없음(생략) → READY → 5/6 신호등 시연',flush=True)
    print('[v19 MAP] 5등 신호 시연 후 Gazebo 창을 닫거나 Ctrl+C를 누르면 종료됩니다.',flush=True)
    proc=subprocess.Popen(['gz','sim','-r','--gui-config',str(gui),str(world)],
                          env=env,start_new_session=True)
    try:
        preview_signal_demo(root,track,env,proc)
        return proc.wait()
    except KeyboardInterrupt:
        print('[v19 MAP] 운영자 중단',flush=True)
        return 130
    finally:
        stop_preview(proc)


def stop_process_group(proc)->None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid,signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=8)
        return
    except subprocess.TimeoutExpired:
        pass
    for sig in (signal.SIGTERM,signal.SIGKILL):
        try:
            os.killpg(proc.pid,sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=3)
            return
        except subprocess.TimeoutExpired:
            continue


def report_status(record:dict)->str:
    return ('PASS' if record.get('finish_time') is not None and
            record.get('laps',0)>=record.get('required_laps',1) else 'INCOMPLETE')


def run_rehearsal(root:Path,stage:str,track:str,ids:list[str],seconds:float,
                  start_mode:str='camera')->int:
    # Do not import rclpy until all static failures are rejected.
    import race_manager as race
    entries=selected_entries(race.load_entries(root),ids,stage)
    preflight_selected(root,ids)
    for e in entries:
        e['_start_mode']=start_mode
        if not (root/'race/install').exists():
            raise ValueError('race/install ROS 워크스페이스가 없습니다. 먼저 bash race_v17.sh prepare')
        prefix=subprocess.run(['ros2','pkg','prefix',e['ros_package']],
                              capture_output=True,text=True,timeout=10)
        if prefix.returncode:
            raise ValueError(e['entry_id']+': ROS 패키지 미설치/환경 미설정: '+e['ros_package'])
    slots=race.grid(root,track)
    if len(slots)<len(entries):
        raise ValueError(f'{track}: 그리드 부족 ({len(slots)}<{len(entries)})')
    cfg=race.read_json(root/'race/event_config.json')
    run_dir=root/'race/previews'/('v17_'+stage+'_'+uuid.uuid4().hex[:12])
    run_dir.mkdir(parents=True,exist_ok=False)
    report_file=run_dir/'report.json'
    event={'event_id':'preview_'+uuid.uuid4().hex[:12], 'track':track,
           'seed':1, 'rehearsal':True}
    data={'mode':'REHEARSAL_NOT_OFFICIAL','stage':stage,'track':track,
          'teams':ids,'required_laps':LAPS[stage],'status':'STARTING',
          'details':{},'official_results_untouched':True}
    data['report_file']=str(report_file)
    save_report(report_file,data)
    monitor=gate=server=bridge=log=env=None
    spawned=[];processes={};error=None;started=False;interrupted=False
    sensor_proc=None;sensor_log=None;sensor_path=run_dir/'sensor_diagnostic.json'
    previous_partition=os.environ.get('GZ_PARTITION')
    print(f'[v17 REHEARSAL] {stage}/{track} / {ids} / 최대 {seconds:.0f}초 / 공식 결과와 분리',flush=True)
    print('[v17 REHEARSAL] 로그: '+str(run_dir),flush=True)
    try:
        server,bridge,env,log=race.start_world(root,track,event,entries,run_dir)
        monitor=race.PoseLapMonitor(track,root,entries,run_dir/'timing.jsonl',cfg)
        gate=race.StartGate(entries)
        # v21 staged ROS-camera-start order
        from race_camera_ready_v21 import wait_gate_ready_before_spawn_v21, check_camera_streams_v21
        print('[v21 START 1/6] 공식 대회 월드·중앙 ROS 브리지 실행 완료',flush=True)
        # 2/6. Launch all team bridges/camera/control nodes BEFORE any spawn.
        for e in entries:
            eid=e['entry_id']
            processes[eid]=race.launch_algo(e,env,run_dir)
        for e in entries:
            eid=e['entry_id']
            wait_gate_ready_before_spawn_v21(gate,eid,processes[eid],timeout=30)
        # 3/6. Spawn only after the ROS control nodes have start subscriptions.
        for idx,e in enumerate(entries):
            eid=e['entry_id']
            print(f'[v21 START 3/6] {eid}: 차량 스폰',flush=True)
            race.spawn(root,track,e,slots[0 if stage in ('vehicles','qualifying') else idx],env,run_dir)
            spawned.append(eid)
            if not monitor.wait_seen(eid):
                raise RuntimeError(eid+': base_link 계측이 준비되지 않았습니다')
        missing=gate.ready(ids,processes,timeout=30)
        if missing:
            raise RuntimeError('출발 신호 구독 또는 참가 알고리즘 준비 실패: '+', '.join(missing))
        # Independently observe actual ROS/Gazebo sensors during the rehearsal.
        # The observer is read-only, runs in its own process and cannot control cars.
        diagnostic=root/'race/scripts/environment_diagnostic_v15.py'
        if diagnostic.is_file():
            sensor_log=(run_dir/'sensors.log').open('w')
            sensor_cmd=[sys.executable,str(diagnostic),'capture',
                '--root',str(root),'--track',track,'--label','competition',
                '--partition',env['GZ_PARTITION'],'--seconds',str(min(600.,max(10.,seconds))),
                '--output',str(sensor_path)]
            for eid in ids:sensor_cmd.extend(['--team',eid])
            sensor_proc=subprocess.Popen(sensor_cmd,env=env,stdout=sensor_log,
                                          stderr=subprocess.STDOUT,start_new_session=True)
            print('[v17] 카메라·LiDAR·IMU·/clock 통신 수집 시작',flush=True)
        else:
            data['sensor_status']='UNAVAILABLE (v15 diagnostic missing)'
        # Same v10 5-red light race start code; results are NOT official.
        delay=float(cfg.get('lights_out_min_delay_seconds',.2))
        # 4/6. Do not light the gantry until real per-team ROS images arrive.
        check_camera_streams_v21(root,run_dir,env,entries,processes,timeout=60)
        print('[v21 START 5/6] 실제 카메라 영상 확인 후 운영자 READY·신호등 실행',flush=True)
        race.start_sequence(track,root,entries,delay,monitor,gate,LAPS[stage],env, operator_ready=True, world_process=server, bridge_process=bridge, stage=stage)
        started=True
        until=time.monotonic()+seconds
        next_line=0.
        while time.monotonic()<until:
            if server.poll() is not None or bridge.poll() is not None:
                raise RuntimeError('Gazebo 서버 또는 ROS 브리지가 비정상 종료됐습니다')
            monitor.health(ids)
            now=time.monotonic()
            if now>=next_line:
                for eid in ids:
                    state=monitor.snapshot(eid)
                    print(f'[v17 주행점검] {eid} / 랩 {state.get("laps",0)}/{LAPS[stage]} / '+
                          monitor.diagnostic_line(eid),flush=True)
                next_line=now+5
            if any(processes[eid].poll() is not None for eid in ids):
                raise RuntimeError('참가 알고리즘 종료: '+', '.join(eid for eid in ids if processes[eid].poll() is not None))
            if all(monitor.snapshot(eid).get('finish_time') is not None for eid in ids):
                print('[v17] 선택 차량 모두 목표 랩 완료',flush=True)
                break
            time.sleep(.05)
        data['status']='OBSERVATION_COMPLETE'
    except KeyboardInterrupt:
        interrupted=True
        data['status']='INTERRUPTED'
        data['error']='운영자 Ctrl+C 종료'
        print('[v17] 운영자 중단: 공식 기록은 변경하지 않습니다.',flush=True)
    except BaseException as exc:
        error=exc
        data['status']='ERROR'
        data['error']=str(exc) or type(exc).__name__
        print('[v17] 점검 오류: '+data['error'],file=sys.stderr,flush=True)
    finally:
        if monitor:
            for eid in ids:
                try:
                    state=monitor.snapshot(eid)
                    row=dict(required_laps=LAPS[stage],laps=state.get('laps',0),
                             finish_time=state.get('finish_time'),
                             max_displacement_m=state.get('max_displacement_m'),
                             speed_messages_after_release=state.get('speed_messages_after_release'),
                             max_abs_speed_command=state.get('max_abs_speed_command'))
                    row['status']=report_status(row)
                    if row['status']!='PASS' and started:
                        try:row['reason']=monitor.failure_reason(eid,'모의주행 미완주')
                        except Exception as exc:row['reason']='상태 상세 조회 오류: '+str(exc)
                    data['details'][eid]=row
                except Exception as exc:
                    data['details'][eid]={'status':'UNKNOWN','reason':'계측 스냅샷 오류: '+str(exc)}
        errors=[]
        # Stop the sensor observer first, while the team and Gazebo are still live.
        if sensor_proc is not None:
            if sensor_proc.poll() is None:
                try:
                    os.killpg(sensor_proc.pid,signal.SIGINT)
                    sensor_proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    stop_process_group(sensor_proc)
                except ProcessLookupError:
                    pass
            if sensor_path.is_file():
                data['sensor_report']=str(sensor_path)
                try:
                    snapshot=json.loads(sensor_path.read_text(encoding='utf-8'))
                    data['sensor_status']='CAPTURED'
                    data['sensor_world_match']=bool(snapshot.get('world_match'))
                    data['sensor_summary']={
                        'clock_samples':snapshot.get('samples',{}).get('/clock',{}).get('count',0),
                        'teams':{eid:snapshot.get('findings',{}).get(eid,[]) for eid in ids}}
                except (ValueError,OSError) as exc:
                    data['sensor_status']='INVALID: '+str(exc)
            else:
                data['sensor_status']='NOT_CAPTURED (see sensors.log)'
        if sensor_log is not None:
            sensor_log.close()
        def safely(label,func):
            try:func()
            except BaseException as exc:errors.append(label+': '+str(exc))
        if gate:
            for eid in ids:
                if eid in processes and gate.pubs[eid].get_subscription_count()>0:
                    safely(eid+' gate off',lambda eid=eid:gate.publish([eid],False))
        for eid,proc in processes.items():
            safely(eid+' stop',lambda proc=proc:race.stop(proc))
        if server and env:
            for eid in spawned:
                safely(eid+' remove',lambda eid=eid:race.remove(track,eid,env))
        if gate:safely('gate',gate.close)
        if monitor:safely('monitor',monitor.close)
        if gate or monitor:
            def shutdown_ros():
                import rclpy
                if rclpy.ok():rclpy.shutdown()
            safely('rclpy',shutdown_ros)
        if bridge:safely('bridge',lambda:race.stop(bridge))
        if server:safely('Gazebo',lambda:race.stop(server))
        if log:safely('log',log.close)
        # ONLY generated files under this unique operator preview directory.
        for e in entries:
            prepared=e.get('_prepared_model')
            if prepared:
                p=Path(prepared).resolve()
                if p.is_relative_to(run_dir.resolve()):
                    safely(e['entry_id']+' generated model',lambda p=p:p.unlink(missing_ok=True))
                else:
                    errors.append(e['entry_id']+': 외부 임시 모델 경로 (삭제 안 함): '+str(p))
        if previous_partition is None:os.environ.pop('GZ_PARTITION',None)
        else:os.environ['GZ_PARTITION']=previous_partition
        if errors:
            data['cleanup_errors']=errors
            data['status']='ERROR'
        if data['status']=='OBSERVATION_COMPLETE' and not all(
                item.get('status')=='PASS' for item in data['details'].values()):
            data['status']='INCOMPLETE'
        save_report(report_file,data)
        if data.get('sensor_status') not in ('CAPTURED',):
            print('[v17 경고] 센서 진단 미완료: '+str(data.get('sensor_status','미실행')),flush=True)
        elif not data.get('sensor_world_match'):
            print('[v17 경고] 진단기의 대상 Gazebo 월드 일치 검증이 불확실합니다.',flush=True)
        print('[v17] 모의주행 결과: '+data['status']+' / '+str(report_file),flush=True)
        for eid,row in data['details'].items():
            print('  '+eid+' '+row.get('status','UNKNOWN')+
                  ' '+row.get('reason',''),flush=True)
    return 130 if interrupted else (0 if data['status']=='OBSERVATION_COMPLETE' else 2)


def save_report(path:Path,data:dict)->None:
    temp=path.with_suffix('.tmp')
    try:
        temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        temp.replace(path)
    finally:
        temp.unlink(missing_ok=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description='URRC F1 v17 대회 운영자의 맵/모의주행 사전점검')
    parser.add_argument('mode',choices=('preview','rehearse'))
    parser.add_argument('--root',type=Path,default=Path.home()/'urrc_f1_ws')
    parser.add_argument('stage',choices=STAGES)
    parser.add_argument('--track',choices=TRACKS)
    parser.add_argument('--team',help='선택 참가팀: team_01')
    parser.add_argument('--teams',help='동시 참가팀: team_01,team_02')
    parser.add_argument('--seconds',type=float,default=120)
    parser.add_argument('--start-mode',choices=('camera','signal'),default='camera')
    a=parser.parse_args(argv)
    root=a.root.expanduser().resolve()
    if not (root/'race/scripts/race_manager.py').is_file():
        parser.error('유효한 URRC F1 대회 운영 워크스페이스가 아닙니다')
    if a.mode=='preview' and (a.team or a.teams):
        parser.error('preview는 차량 없는 맵 미리보기입니다. 차량 포함은 rehearse를 사용하세요')
    if a.mode=='rehearse' and not 5<=a.seconds<=2400:
        parser.error('--seconds는 5~2400초 범위여야 합니다')
    if a.team and a.teams:
        parser.error('--team과 --teams는 동시에 지정할 수 없습니다')
    names=[a.team] if a.team else ([x.strip() for x in a.teams.split(',')] if a.teams else [])
    if a.mode=='rehearse':
        try:validate_team_names(names)
        except ValueError as exc:parser.error(str(exc))
    try:
        track,origin=choose_track(root,a.stage,a.track)
        print('[v17 MAP] '+track+' / '+origin,flush=True)
        if a.mode=='preview':
            return preview_map(root,a.stage,track)
        return run_rehearsal(root,a.stage,track,names,a.seconds,a.start_mode)
    except KeyboardInterrupt:
        print('[v17] 운영자 중단',flush=True)
        return 130
    except (ValueError,OSError,RuntimeError,subprocess.TimeoutExpired,subprocess.CalledProcessError) as exc:
        print('[v17] 점검 실행 실패: '+str(exc),file=sys.stderr)
        return 2

if __name__=='__main__':sys.exit(main())
