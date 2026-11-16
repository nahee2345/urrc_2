#!/usr/bin/env python3
"""Read-only Gazebo <-> ROS 2 <-> participant contract diagnostics.

Run alongside a live race or practice Gazebo. No team code is imported/edited.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path

SENSOR_KINDS = {
    'sensor_msgs/msg/Image': 'camera',
    'sensor_msgs/msg/CompressedImage': 'camera',
    'sensor_msgs/msg/LaserScan': 'lidar',
    'sensor_msgs/msg/PointCloud2': 'lidar',
    'sensor_msgs/msg/Imu': 'imu',
    'tf2_msgs/msg/TFMessage': 'pose',
    'nav_msgs/msg/Odometry': 'odom',
    'geometry_msgs/msg/Twist': 'drive',
    'geometry_msgs/msg/TwistStamped': 'drive',
}
OBSERVATION_TYPES = {**SENSOR_KINDS,
    'rosgraph_msgs/msg/Clock': 'clock',
    'std_msgs/msg/Bool': 'gate',
    'std_msgs/msg/Float32': 'command',
    'std_msgs/msg/Float64': 'command',
}


def team_number(team: str) -> str:
    match = re.fullmatch(r'team_(\d{2})', team)
    if not match:
        raise ValueError(f'잘못된 참가팀 ID: {team}')
    return match.group(1)


def topic_is_team(topic: str, team: str, entry: dict) -> bool:
    """Only strong matches; /team01/ alone can belong to TEAM 02 too."""
    suffix = team_number(team)
    if topic in (entry.get('speed_topic'), entry.get('steering_topic'),
                 entry.get('lidar_topic'), f'/race/start/{team}',
                 f'/model/{team}/urrc_timing'):
        return True
    # Explicit team index or name, not ambiguous shared namespace.
    return bool(re.search(rf'(?<!\d){re.escape(suffix)}(?:$|/)', topic) and
                re.search(rf'(?:_|/){re.escape(suffix)}(?:$|/)', topic)) or bool(
                    re.search(rf'(?<![a-z\d])team_?{int(suffix)}(?:/|_|$)', topic))


def load_teams(root: Path, names: list[str] | None = None) -> dict:
    found = {}
    requested = set(names or [])
    for path in sorted((root / 'src/team').glob('team_??/race_entry.json')):
        team = path.parent.name
        if requested and team not in requested:
            continue
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (ValueError, OSError) as exc:
            found[team] = {'_error': str(exc)}
            continue
        found[team] = data
    for team in requested - set(found):
        found[team] = {'_error': 'src/team 제출물 없음'}
    return found


def select_gz_process(matches: list[dict], label: str) -> tuple[str | None, str]:
    """Choose the intended world, never silently substitute a race for practice."""
    wanted = 'practice' if label == 'practice' else 'race'
    choices = [m for m in matches if m['world_arg'] == wanted and m.get('partition')]
    partitions = set(m['partition'] for m in choices)
    if len(partitions) == 1:
        return next(iter(partitions)), 'matched'
    if len(partitions) > 1:
        return None, 'ambiguous'
    return None, 'not_found'


def discover_gz_partition(track: str, label: str = 'competition') -> tuple[str | None, list[dict], str]:
    """Inspect process metadata only; don't expose unrelated environment vars."""
    matches = []
    try:
        folders = list(Path('/proc').iterdir())
    except OSError:
        return None, [], 'proc_unavailable'
    for folder in folders:
        if not folder.name.isdigit():
            continue
        try:
            cmd = (folder / 'cmdline').read_bytes().replace(b'\0', b' ').decode('utf-8', 'replace')
            # An actual world file in the simulator command is required.
            race = bool(re.search(rf'(?<![\w.]){re.escape(track)}_race\.sdf(?:\s|$)', cmd))
            practice = bool(re.search(rf'(?<![\w.]){re.escape(track)}\.sdf(?:\s|$)', cmd))
            if not race and not practice:
                continue
            partition = None
            for value in (folder / 'environ').read_bytes().split(b'\0'):
                if value.startswith(b'GZ_PARTITION='):
                    partition = value.split(b'=', 1)[1].decode('utf-8', 'replace')
                    break
            matches.append({'pid': int(folder.name), 'partition': partition,
                            'world_arg': 'race' if race else 'practice'})
        except (OSError, PermissionError):
            continue
    selected, state = select_gz_process(matches, label)
    return selected, matches, state


def gazebo_topics(partition: str | None) -> dict:
    env = os.environ.copy()
    if partition:
        env['GZ_PARTITION'] = partition
    try:
        proc = subprocess.run(['gz', 'topic', '-l'], capture_output=True,
                              text=True, env=env, timeout=7)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {'error': str(exc), 'topics': []}
    if proc.returncode:
        return {'error': (proc.stderr or proc.stdout)[-350:], 'topics': []}
    return {'topics': sorted(set(t.strip() for t in proc.stdout.splitlines() if t.strip()))}


def topic_boundary(graph: dict, name: str) -> dict:
    info = graph.get(name) or {}
    pub = info.get('publishers', 0)
    external = info.get('subscribers_excluding_diagnostic', info.get('subscribers', 0))
    return {'published': pub > 0, 'participant_subscribed': external > 0,
            'qos_compatible': info.get('qos_compatible'),
            'qos_state': info.get('qos_state', 'unknown')}


def snapshot_clock_status(samples: dict) -> dict:
    clock = samples.get('/clock', {})
    count = int(clock.get('count', 0))
    lo = clock.get('min_value')
    hi = clock.get('max_value')
    progress = (hi-lo) if isinstance(hi, (int,float)) and isinstance(lo, (int,float)) else None
    status = 'PASS' if count > 1 and progress is not None and progress > 0 else 'CHECK'
    return {'status': status, 'messages': count, 'advanced_sim_seconds': progress}


def classify_team(state: dict, samples: dict, team: str) -> list[dict]:
    """Diagnostic hints, not proof of root cause or camera recognition."""
    e = state['teams'][team]
    if '_error' in e:
        return [{'status': 'FAIL', 'area': 'registration', 'detail': e['_error']}]
    gate = samples.get(f'/race/start/{team}', {})
    speed = samples.get(e.get('speed_topic'), {})
    steer = samples.get(e.get('steering_topic'), {})
    pose = samples.get(f'/model/{team}/urrc_timing', {})
    related = [(name, s) for name, s in samples.items() if topic_is_team(name, team, e)]
    cam = [s for name, s in related if s.get('kind') == 'camera']
    lidar = [s for name, s in related if s.get('kind') == 'lidar']
    imu = [s for name, s in related if s.get('kind') == 'imu']
    drive = [s for name, s in related if s.get('kind') == 'drive']
    results = []
    graph = state.get('ros_topics', {})
    def add(area, status, detail):
        results.append({'area': area, 'status': status, 'detail': detail})
    if not state.get('world_match', False):
        add('environment', 'CHECK', '요청한 연습/대회 월드 프로세스와 Gazebo 파티션을 확인하지 못했습니다. 다른 환경 데이터가 혼합됐을 수 있습니다.')
    add('gate', 'PASS' if gate.get('count', 0) else 'UNKNOWN',
        f'출발 신호 관측={gate.get("count",0)}, 마지막 값={gate.get("last_value")} (수신 확인과 앱 적용은 별개)')
    for area, group in [('camera', cam), ('lidar', lidar), ('imu', imu)]:
        receiving = [x for x in group if x.get('count', 0) > 0]
        add(area, 'PASS' if receiving else 'UNKNOWN',
            'ROS 수신=' + str(sum(x.get('count',0) for x in group)) +
            ' 프레임, 토픽=' + ', '.join(x.get('topic','?') for x in group))
        # A diagnostic subscription does not prove a team subscriber exists.
        if receiving and not any(topic_boundary(graph,x['topic'])['participant_subscribed'] for x in receiving):
            add(area+'_consumer', 'CHECK', 'ROS 수신은 확인되지만 진단기 외 구독자를 찾지 못했습니다. 참가 노드 구독 토픽을 확인하십시오.')
        for x in receiving:
            boundary=topic_boundary(graph,x['topic'])
            if boundary['qos_state'] == 'incompatible':
                add(area+'_qos','CHECK',x['topic']+': 발행자와 참가 구독자 QoS 비호환 가능성')
    add('timing_pose', 'PASS' if pose.get('count',0) else 'UNKNOWN',
        f'중앙 계측 포즈 수신={pose.get("count",0)}')
    add('speed_steering', 'PASS' if speed.get('count',0) and steer.get('count',0) else 'UNKNOWN',
        f'속도={speed.get("count",0)}회 (최대={speed.get("max_abs_value",0)}), 조향={steer.get("count",0)}회')
    if speed.get('count',0) and speed.get('max_abs_value',0) <= .05:
        add('cause_hint', 'CHECK', '속도 명령이 0입니다. 카메라 소등 인식·출발 게이트·팀 안전조건을 조사하십시오.')
    elif speed.get('max_abs_value',0) > .05 and not any(s.get('count',0) for s in drive):
        add('cause_hint', 'CHECK', '속도 명령은 있지만 Gazebo 구동 명령 수신이 관측되지 않았습니다. 어댑터·브리지를 확인하십시오.')
    else:
        add('drive_path', 'PASS' if any(s.get('count',0) for s in drive) else 'UNKNOWN',
            f'구동 명령 수신={sum(s.get("count",0) for s in drive)}')
    # Samples do not prove that the team algorithm subscribed to the stream.
    return results


def compare_snapshots(a: dict, b: dict) -> dict:
    topics_a = a.get('ros_topics', {})
    topics_b = b.get('ros_topics', {})
    result = {'practice': a.get('label'), 'race': b.get('label'), 'teams': {}, 'warnings': []}
    if a.get('label') != 'practice' or b.get('label') != 'competition':
        result['warnings'].append('첫 보고서는 practice, 두 번째는 competition이어야 합니다.')
    if not a.get('world_match') or not b.get('world_match'):
        result['warnings'].append('월드 선택 미확인: 비교 결과를 실제 환경 차이로 확정하지 마십시오.')
    if a.get('track') != b.get('track'):
        result['warnings'].append('비교 트랙이 서로 다릅니다.')
    if a.get('ros_domain_id') != b.get('ros_domain_id'):
        result['warnings'].append('연습·대회 ROS_DOMAIN_ID가 다릅니다. 별도 도메인에서 수집했는지 확인하십시오.')
    result['clock'] = {'practice': snapshot_clock_status(a.get('samples',{})),
                       'competition': snapshot_clock_status(b.get('samples',{}))}
    for team in sorted(set(a.get('teams',{})) | set(b.get('teams',{}))):
        ea = a.get('teams',{}).get(team,{})
        eb = b.get('teams',{}).get(team,{})
        score = {}
        for group in ['camera','lidar','imu','pose','odom','gate','command','drive']:
            def hits(s, meta):
                return sorted(name for name, v in s.get('samples',{}).items()
                              if v.get('kind') == group and topic_is_team(name,team,meta)
                              and v.get('count',0)>0)
            left = hits(a,ea);right = hits(b,eb)
            score[group] = {'practice_live':left,'race_live':right,
                            'missing_in_race':bool(left) and not bool(right)}
        result['teams'][team]=score
    result['gazebo']={
        'practice_worlds':sorted(t for t in a.get('gazebo',{}).get('topics',[]) if t.startswith('/world/'))[:25],
        'race_worlds':sorted(t for t in b.get('gazebo',{}).get('topics',[]) if t.startswith('/world/'))[:25],
    }
    return result


def run_capture(args):
    try:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        print('ROS 2 Python 환경을 source 해야 합니다: '+str(exc), file=sys.stderr)
        return 2
    root=args.root.resolve()
    teams=load_teams(root, args.team)
    detected, process_details, discovery=discover_gz_partition(args.track,args.label)
    partition=args.partition or detected
    world_match=bool(detected and partition == detected and discovery == 'matched')
    if not partition:
        print('[v15 CHECK] 대상 Gazebo 월드를 확인하지 못했습니다. '
              '--partition을 명시하면 수집할 수 있지만 월드 일치 여부는 별도 검증해야 합니다.', file=sys.stderr)
        # Do not list Gazebo topics from an unrelated default partition.
        gz_before={'error':'target Gazebo partition unknown','topics':[]}
    else:
        gz_before=gazebo_topics(partition)
    rclpy.init()
    node=rclpy.create_node('urrc_readonly_diagnostic_v15')
    samples={}
    subscriptions={}
    topic_types={}
    started=time.monotonic()
    last_scan=0.0
    try:
        while time.monotonic()-started < args.seconds:
            now=time.monotonic()
            if now-last_scan > 1.0:
                topic_types={name:sorted(types) for name,types in node.get_topic_names_and_types()}
                needed={'/clock':'rosgraph_msgs/msg/Clock'}
                for team, entry in teams.items():
                    needed[f'/race/start/{team}']='std_msgs/msg/Bool'
                    needed[f'/model/{team}/urrc_timing']='tf2_msgs/msg/TFMessage'
                    for field in ['speed_topic','steering_topic']:
                        if entry.get(field):
                            advertised=topic_types.get(entry[field],[])
                            needed[entry[field]]=next((t for t in advertised if t in ('std_msgs/msg/Float32','std_msgs/msg/Float64')),
                                                           'std_msgs/msg/Float32')
                for name, kinds in topic_types.items():
                    if name.startswith('/rosout') or name.startswith('/parameter_events'):
                        continue
                    if any(topic_is_team(name,team,e) for team,e in teams.items() if '_error' not in e):
                        for kind in kinds:
                            if kind in SENSOR_KINDS:
                                needed[name]=kind
                for name, kind in sorted(needed.items()):
                    if name in subscriptions:
                        continue
                    if kind not in OBSERVATION_TYPES:
                        continue
                    try:
                        msgtype=get_message(kind)
                    except (ValueError, ImportError, AttributeError):
                        continue
                    sample={'topic':name,'type':kind,'kind':OBSERVATION_TYPES[kind],
                            'count':0,'first_wall':None,'last_wall':None,
                            'last_value':None,'max_abs_value':0.0,
                            'latest_stamp_sec':None,'frame_examples':[],
                            'latest_stamp_age_sim_seconds':None,
                            'min_value':None,'max_value':None}
                    samples[name]=sample
                    def on_msg(msg, sample=sample):
                        t=time.monotonic()
                        sample['count']+=1
                        sample['first_wall']=sample['first_wall'] or t
                        sample['last_wall']=t
                        if sample['kind']=='clock':
                            value=msg.clock.sec + msg.clock.nanosec/1e9
                            sample['last_value']=value
                            sample['min_value']=(value if sample['min_value'] is None else min(sample['min_value'],value))
                            sample['max_value']=(value if sample['max_value'] is None else max(sample['max_value'],value))
                        elif sample['kind'] in ('command','gate'):
                            value=getattr(msg,'data',None)
                            sample['last_value']=value
                            if isinstance(value, (float,int)) and not isinstance(value, bool) and math.isfinite(value):
                                sample['max_abs_value']=max(sample['max_abs_value'],abs(value))
                        elif sample['kind']=='drive':
                            twist=msg.twist if hasattr(msg,'twist') else msg
                            value=float(twist.linear.x)
                            sample['last_value']=value
                            if math.isfinite(value):sample['max_abs_value']=max(sample['max_abs_value'],abs(value))
                        stamp=None
                        if hasattr(msg,'header'):
                            stamp=msg.header.stamp
                        elif sample['kind']=='pose':
                            for tf in getattr(msg,'transforms',[]):
                                if hasattr(tf,'header') and (tf.header.stamp.sec or tf.header.stamp.nanosec):
                                    stamp=tf.header.stamp
                                    break
                        if stamp is not None and (stamp.sec or stamp.nanosec):
                            stamp_sec=stamp.sec+stamp.nanosec/1e9
                            sample['latest_stamp_sec']=stamp_sec
                            clock_at_receive=samples.get('/clock',{}).get('last_value')
                            sample['latest_stamp_age_sim_seconds']=(round(clock_at_receive-stamp_sec,3)
                                if isinstance(clock_at_receive,(int,float)) else None)
                        if sample['kind']=='pose' and len(sample['frame_examples'])<8:
                            for tf in getattr(msg,'transforms',[]):
                                name=tf.child_frame_id
                                if name and name not in sample['frame_examples']:
                                    sample['frame_examples'].append(name)
                    # Best effort sensor-data QoS receives RELIABLE and BEST_EFFORT streams.
                    subscriptions[name]=node.create_subscription(msgtype,name,on_msg,qos_profile_sensor_data)
                last_scan=now
            rclpy.spin_once(node,timeout_sec=.10)
    except KeyboardInterrupt:
        print('[v15] 측정 중단: 수집된 데이터로 보고서 작성')
    finally:
        graph={}
        for name,types in topic_types.items():
            info={'types':types, 'publishers':node.count_publishers(name),
                  'subscribers_excluding_diagnostic':max(0,node.count_subscribers(name)-int(name in subscriptions)),
                  'qos_state':'unknown','qos_compatible':None}
            try:
                publishers=node.get_publishers_info_by_topic(name)
                consumers=[x for x in node.get_subscriptions_info_by_topic(name)
                           if x.node_name != node.get_name()]
                info['publishers']=len(publishers)
                info['subscribers_excluding_diagnostic']=len(consumers)
                if publishers and consumers:
                    from rclpy.qos import qos_check_compatible, QoSCompatibility
                    statuses=[qos_check_compatible(pub.qos_profile,sub.qos_profile).compatibility
                              for pub in publishers for sub in consumers]
                    if QoSCompatibility.OK in statuses:
                        info['qos_state']='compatible';info['qos_compatible']=True
                    elif all(x==QoSCompatibility.ERROR for x in statuses):
                        info['qos_state']='incompatible';info['qos_compatible']=False
                    else:
                        info['qos_state']='uncertain'
                elif not consumers:
                    info['qos_state']='no_external_subscriber'
            except (AttributeError,TypeError,ValueError,RuntimeError):
                pass
            graph[name]=info
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    gz_after=gazebo_topics(partition) if partition else {'error':'target Gazebo partition unknown','topics':[]}
    snapshot={'version':15,'label':args.label,'track':args.track,
              'created_local':dt.datetime.now().astimezone().isoformat(timespec='seconds'),
              'seconds':round(time.monotonic()-started,2),
              'gz_partition':partition,'processes':process_details,
              'world_match':world_match,'partition_discovery':discovery,
              'ros_domain_id':os.environ.get('ROS_DOMAIN_ID','default'),
              'teams':teams,'ros_topics':graph,
              'gazebo':gz_after if gz_after.get('topics') else gz_before,
              'samples':samples}
    for s in samples.values():
        start,end=s.get('first_wall'),s.get('last_wall')
        s['frequency_hz']=round((s['count']-1)/(end-start),2) if start is not None and end is not None and end>start else None
        # Age was measured at message receipt, not by subtracting the last
        # clock at end of capture (which creates a false latency alarm).
        s['stamp_lag_sim_seconds']=s.get('latest_stamp_age_sim_seconds')
        # Normalize wall timestamps to relative to capture start.
        for field in ('first_wall','last_wall'):
            if s.get(field) is not None:s[field]=round(s[field]-started,3)
    snapshot['findings']={team:classify_team(snapshot,samples,team) for team in teams}
    outfile=args.output or (root/'race/results/diagnostics'/
                    ('env_'+args.label+'_'+dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json'))
    outfile.parent.mkdir(parents=True,exist_ok=True)
    outfile.write_text(json.dumps(snapshot,ensure_ascii=False,indent=2,default=str)+'\n',encoding='utf-8')
    print(f'\n[v15] 측정={snapshot["seconds"]}초, 트랙={args.track}, GZ_PARTITION={partition or "확인불가"}')
    clock_health=snapshot_clock_status(samples)
    print(f'[v15] /clock 수신={clock_health["messages"]}, '
          f'진행={clock_health["advanced_sim_seconds"]}초, 시계상태={clock_health["status"]}')
    if not world_match:
        print(f'[v15 CHECK] 요청 월드={args.label}, 탐색상태={discovery}, '
              '수집 중인 데이터가 목표 환경에서 온 것인지 확정할 수 없습니다.')
    print(f'[v15] Gazebo 토픽={len(snapshot["gazebo"].get("topics",[]))}, ROS 토픽={len(graph)}')
    if not partition:
        print('[v15 CHECK] Gazebo 파티션을 자동 확인하지 못했습니다. 필요하면 --partition을 지정하십시오.')
    for team,e in teams.items():
        raw=[t for t in snapshot['gazebo'].get('topics',[]) if topic_is_team(t,team,e)]
        print(f'[v15 GZ] {team} 관련 토픽={raw[:18]} (존재 확인이며 발행 주기 검증 아님)')
        for name,ty in graph.items():
            if topic_is_team(name,team,e):
                print(f'[v15 ROS] {name} publishers={ty["publishers"]}, '
                      f'subscribers_excluding_diagnostic={ty["subscribers_excluding_diagnostic"]} qos={ty["qos_state"]}')
        raw_camera=[name for name in raw if any(x in name.lower() for x in ['image','camera','rgb'])]
        ros_camera=[name for name,s in samples.items() if s.get('kind')=='camera'
                    and topic_is_team(name,team,e) and s.get('count',0)>0]
        if raw_camera and not ros_camera:
            print(f'[v15 CHECK] {team}: Gazebo 카메라 토픽 발견, ROS 영상 미수신. '
                  '브리지·QoS·토픽명을 검사하십시오.')
    if not teams:
        print('[v15] 등록된 팀 없음: src/team/team_??/race_entry.json을 넣은 다음 팀별 진단을 수행하십시오.')
    for team, findings in snapshot['findings'].items():
        print('\n'+team)
        for item in findings:print('  '+item['status']+' '+item['area']+': '+item['detail'])
        e=teams[team]
        for name,s in samples.items():
            if topic_is_team(name,team,e) and s.get('count',0)>0:
                print(f'    {name}: {s["count"]}회 / {s["frequency_hz"]}Hz'+
                      (f' / sim 지연={s["stamp_lag_sim_seconds"]}초' if s['stamp_lag_sim_seconds'] is not None else '')+
                      (f' / TF={s["frame_examples"]}' if s.get('frame_examples') else ''))
    print(f'\n[v15] 진단 결과: {outfile}')
    print('[v15] PASS=ROS 관측 성공일 뿐, 팀 애플리케이션의 실제 처리·물리 주행 성공은 별도 검증입니다.')
    return 0


def main(argv=None):
    p=argparse.ArgumentParser(description='URRC practice/race environment diagnostic; read-only')
    sub=p.add_subparsers(dest='cmd',required=True)
    c=sub.add_parser('capture',help='Read ROS2/Gazebo during a running practice or race')
    c.add_argument('--root',type=Path,default=Path.home()/'urrc_f1_ws')
    c.add_argument('--team',action='append',choices=[f'team_{i:02d}' for i in range(1,13)])
    c.add_argument('--track',choices=('monza','spa','monaco','cota','hungaroring','baku'),default='monza')
    c.add_argument('--seconds',type=float,default=40)
    c.add_argument('--label',choices=('practice','competition'),default='competition')
    c.add_argument('--partition')
    c.add_argument('--output',type=Path)
    d=sub.add_parser('compare',help='Compare previously captured JSON reports')
    d.add_argument('practice',type=Path)
    d.add_argument('competition',type=Path)
    d.add_argument('--output',type=Path)
    args=p.parse_args(argv)
    if args.cmd=='capture':
        if not 3<=args.seconds<=600:
            p.error('--seconds must be within 3..600')
        return run_capture(args)
    a=json.loads(args.practice.read_text(encoding='utf-8'))
    b=json.loads(args.competition.read_text(encoding='utf-8'))
    comparison=compare_snapshots(a,b)
    out=args.output or Path.cwd()/'urrc_env_comparison_v15.json'
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(comparison,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for warning in comparison['warnings']:
        print('[v15 CHECK]',warning)
    print('[v15] /clock:',comparison['clock'])
    for team,kinds in comparison['teams'].items():
        print('\n'+team)
        for kind,v in kinds.items():
            if v['missing_in_race']:
                print(f'  CHECK: 연습 환경 {kind} 수신 → 대회 수신 없음')
    print('비교 보고서:',out)
    return 0


if __name__=='__main__':
    try:sys.exit(main())
    except (OSError,ValueError) as exc:
        print('[v15] 오류:',exc,file=sys.stderr)
        sys.exit(2)
