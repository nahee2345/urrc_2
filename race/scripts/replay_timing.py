#!/usr/bin/env python3
"""Recompute an official run from its saved simulation poses, without ROS."""
import argparse
import json
from pathlib import Path
from timing import LapTimer


def replay(folder):
    files=[p for p in folder.glob('*.json') if p.stem in ('qualifying','final')]
    if len(files)!=1:
        raise ValueError('race/results/runs/<실행ID> 폴더를 지정하세요.')
    result=json.loads(files[0].read_text())
    if not result.get('valid'):
        raise ValueError('무효/미완료 실행입니다. 정상 기록으로 재확인할 수 없습니다.')
    cfg=result['timing_config'];timers={};gates=None
    for line in (folder/'timing.jsonl').read_text().splitlines():
        event=json.loads(line)
        if event['type']=='gates':gates=event['gates']
        elif event['type']=='arm':
            timer=LapTimer(gates,cfg['max_pose_gap_seconds'],cfg['max_pose_step_m'])
            timer.update(event['previous']);timer.arm(event['start_time'],event['laps'])
            timers[event['entry_id']]=timer
        elif event['type']=='pose':
            timers[event['entry_id']].update(event['sample'])
    for row in result['results']:
        timer=timers.get(row['entry_id'])
        actual=timer.snapshot() if timer else {'laps':0,'lap_times_seconds':[],'finish_time':None}
        for key,source in [('laps','laps'),('lap_times_seconds','lap_times_seconds'),('finish_sim_time','finish_time')]:
            if row[key]!=actual[source]:
                raise ValueError(f"재계측 불일치: {row['entry_id']} {key}")
    print(f"PASS: {len(result['results'])}대 랩 수·각 랩타임·완주 시각 재계측 일치")
    if result.get('provisional'):
        print('완주 시각 계측 구간이 겹칩니다. 순위는 판정 필요 상태입니다.')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run_directory',type=Path)
    replay(p.parse_args().run_directory)
