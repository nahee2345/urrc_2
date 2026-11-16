#!/usr/bin/env python3
"""Read-only ROS image-frame preflight before F1 start lights (ROS2 Jazzy).

No participant model, nodes, or official result files are changed.
Image frames must be attributable to each selected team; no global guess.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

IMAGE_TYPES = {'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'}
ID_RE = re.compile(r'^team_(0[1-9]|1[0-2])$')


def team_topic_score(team: str, topic: str) -> int:
    """Use explicit team namespace or suffix; never confuse team_01/11."""
    match = ID_RE.fullmatch(team)
    if not match:
        raise ValueError('유효하지 않은 팀 ID: ' + team)
    n = match.group(1)
    path = topic.lower()
    score = 0
    if re.search(rf'(^|/)(team{n}|team_{n})(/|$)', path):
        score += 8
    if re.search(rf'_{n}(/|$)', path):
        score += 7
    if not score:
        return 0
    if 'start_image' in path or 'start_camera' in path:
        score += 3
    if 'image' in path or 'camera' in path:
        score += 1
    return score


def choose_topics(topics: dict[str, list[str]], teams: list[str], explicit=None):
    """Pick one per-team ROS image topic; explicit operator overrides win."""
    explicit = explicit or {}
    chosen = {}
    for team in teams:
        if team in explicit:
            topic = explicit[team]
            if not isinstance(topic,str) or not topic.startswith('/'):
                raise ValueError(team + ': 지정 카메라 토픽이 절대 ROS 이름이 아닙니다')
            if not IMAGE_TYPES.intersection(topics.get(topic, [])):
                continue  # Topic may be late: retry until timeout.
            chosen[team] = topic
            continue
        ranked = sorted(((team_topic_score(team,t),t) for t,ty in topics.items()
                         if IMAGE_TYPES.intersection(ty)),key=lambda x:(-x[0],x[1]))
        if ranked and ranked[0][0]>0:
            chosen[team] = ranked[0][1]
    return chosen


def load_operator_topics(root, teams):
    """Optional operator-only file; never modify team submissions."""
    path = Path(root)/'race/camera_topics_v21.json'
    if not path.is_file():
        return {}
    data = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(data,dict) or any(not ID_RE.fullmatch(k) for k in data):
        raise ValueError('운영자 카메라 설정에는 team_01~team_12만 사용할 수 있습니다')
    for team,topic in data.items():
        if not isinstance(topic,str) or not topic.startswith('/'):
            raise ValueError(team+': 잘못된 카메라 토픽 설정')
    return {team:topic for team,topic in data.items() if team in teams}


def wait_gate_ready_before_spawn_v21(gate, eid, process, timeout=30):
    """Controller and race-start subscriber must exist before the model spawns."""
    print(f'[v21 START 2/6] {eid}: ROS 브리지·제어 노드 준비 확인',flush=True)
    missing = gate.ready([eid], {eid:process}, timeout=timeout)
    if missing or process.poll() is not None:
        raise RuntimeError(f'{eid}: 차량 스폰 전 제어 노드/출발 구독 미준비 또는 종료')
    # Explicitly hold the vehicle stopped before its model becomes dynamic.
    gate.publish([eid],False)
    print(f'[v21 START 2/6] {eid}: 출발 구독·정지 명령 준비 PASS',flush=True)


def check_camera_streams_v21(root,run_dir,env,entries,processes,timeout=60):
    """Run isolated ROS probe: verify 2 real image frames per team before lights."""
    teams=[e['entry_id'] if isinstance(e,dict) else e for e in entries]
    if not teams:
        raise RuntimeError('카메라 준비 확인 대상 차량이 없습니다')
    for team in teams:
        if not ID_RE.fullmatch(team):
            raise ValueError('유효하지 않은 참가팀: '+team)
        proc=processes.get(team)
        if proc is None or proc.poll() is not None:
            raise RuntimeError(team+': 제어 노드 실행 중단; 신호등 점등 금지')
    if not isinstance(env,dict) or not env.get('GZ_PARTITION'):
        raise RuntimeError('Gazebo 파티션을 찾지 못했습니다')
    output=Path(run_dir)/('camera_ready_v21_'+'-'.join(teams)+'.json')
    script=Path(root)/'race/scripts/race_camera_ready_v21.py'
    if not script.is_file():
        raise RuntimeError('카메라 준비 검사기 파일 누락: '+str(script))
    cmd=[sys.executable,str(script),'--root',str(root),'--timeout',str(int(timeout)),
         '--output',str(output),'--teams',','.join(teams)]
    print('[v21 START 4/6] 차량별 실제 카메라 프레임 2개 이상 수신 대기: '+', '.join(teams),flush=True)
    try:
        result=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=timeout+12,check=False)
    except (OSError,subprocess.TimeoutExpired) as exc:
        raise RuntimeError('카메라 진단 실행/시간 초과: '+str(exc)) from exc
    if result.stdout.strip():
        print(result.stdout.strip(),flush=True)
    if result.returncode!=0:
        message=result.stderr.strip() or result.stdout.strip() or '카메라 프레임 확인 실패'
        raise RuntimeError('출발 전 카메라 미준비: '+message)
    for team in teams:
        if processes[team].poll() is not None:
            raise RuntimeError(team+': 카메라 검사 도중 제어 노드 종료')
    if not output.is_file():
        raise RuntimeError('카메라 검사 결과 누락')
    data=json.loads(output.read_text(encoding='utf-8'))
    if any(data.get('frames',{}).get(team,0)<2 for team in teams):
        raise RuntimeError('카메라 검사 결과 프레임 부족')
    print('[v21 START 4/6] 카메라 영상 수신 PASS',flush=True)
    return data


def capture(teams, root, seconds, output):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, CompressedImage
    from rosgraph_msgs.msg import Clock

    explicit=load_operator_topics(root, teams)
    rclpy.init()
    node=rclpy.create_node('urrc_camera_ready_v21_'+str(os.getpid()))
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    frames={t:0 for t in teams}
    chosen={}
    subscribed={}
    last_stamp={}
    clock_count=[0]
    clock_last=[None]
    def clock_cb(msg):
        now=msg.clock.sec+msg.clock.nanosec/1e9
        if clock_last[0] is None or now>clock_last[0]:
            clock_count[0]+=1
        clock_last[0]=now
    clock_sub=node.create_subscription(Clock,'/clock',clock_cb,qos_profile_sensor_data)
    start=time.monotonic()
    next_discovery=start
    try:
        while time.monotonic()-start<seconds:
            now=time.monotonic()
            if now>=next_discovery:
                listing={name:types for name,types in node.get_topic_names_and_types()}
                found=choose_topics(listing,teams,explicit)
                for team,topic in found.items():
                    if team in subscribed:
                        continue
                    types=listing.get(topic,[])
                    kind=CompressedImage if 'sensor_msgs/msg/CompressedImage' in types else Image
                    def on_frame(msg,team=team):
                        good=bool(getattr(msg,'data',b''))
                        if hasattr(msg,'width'):
                            good=good and int(msg.width)>0 and int(msg.height)>0
                        if good:
                            frames[team]+=1
                            stamp=getattr(getattr(msg,'header',None),'stamp',None)
                            if stamp is not None:
                                last_stamp[team]=stamp.sec+stamp.nanosec/1e9
                    subscribed[team]=node.create_subscription(kind,topic,on_frame,qos_profile_sensor_data)
                    chosen[team]=topic
                    print(f'[v21 CAMERA] {team}: ROS {topic} 확인 대기',flush=True)
                next_discovery=now+.5
            executor.spin_once(timeout_sec=.1)
            if all(frames[t]>=2 for t in teams) and clock_count[0]>=2:
                break
        complete=all(frames[t]>=2 for t in teams) and clock_count[0]>=2
        evidence=dict(complete=complete,teams=teams,topics=chosen,frames=frames,
                      clock_progress_samples=clock_count[0],last_image_sim_stamp=last_stamp,
                      elapsed_wall_seconds=round(time.monotonic()-start,3))
        Path(output).parent.mkdir(parents=True,exist_ok=True)
        Path(output).write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        if complete:
            print('[v21 CAMERA] /clock 진행 및 팀별 카메라 프레임 수신 PASS',flush=True)
            return 0
        missing=[t for t in teams if frames[t]<2]
        print('카메라 영상 수신 실패: '+','.join(missing)+
              f'; ROS 이미지 토픽={chosen}; /clock 진행={clock_count[0]}',file=sys.stderr,flush=True)
        return 2
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser=argparse.ArgumentParser()
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--teams',required=True)
    parser.add_argument('--timeout',type=float,default=60)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    teams=args.teams.split(',')
    if not teams or any(not ID_RE.fullmatch(t) for t in teams):
        parser.error('팀은 team_01~team_12만 허용됩니다')
    if args.timeout<2 or args.timeout>180:
        parser.error('--timeout은 2~180초 범위')
    try:
        return capture(teams,args.root,args.timeout,args.output)
    except (OSError,ValueError,RuntimeError) as e:
        print('카메라 준비 검사 오류: '+str(e),file=sys.stderr)
        return 2


if __name__=='__main__':sys.exit(main())
