#!/usr/bin/env python3
"""URRC standalone admin CLI (ROS-independent validation, Gazebo service control).

This file is an ADD-ON. Existing race_manager integration is required for
scoring, telemetry, sensor access enforcement, and full event resumption.
"""
import argparse
import csv
import datetime as dt
import json
import os
from pathlib import Path
import random
import re
import subprocess
import sys
import tempfile
import time

TRACKS = ('monza', 'spa', 'monaco', 'cota', 'hungaroring', 'baku')
ENTRY_RE = re.compile(r'^team_(0[1-9]|1[0-2])$')


def load_csv(path, required):
    with Path(path).open(encoding='utf-8-sig', newline='') as f:
        rows = list(csv.DictReader(line for line in f if line.strip() and not line.lstrip().startswith('#')))
    if not rows or not set(required).issubset(rows[0]):
        raise ValueError(f'{path}: 필요한 열: {", ".join(required)}')
    return rows


def validate_order(path, count=None):
    rows = load_csv(path, ('grid', 'team'))
    slots = []
    teams = []
    for r in rows:
        slot = int(r['grid'])
        team = r['team'].strip()
        if slot < 1 or slot > 12 or not ENTRY_RE.fullmatch(team):
            raise ValueError(f'유효하지 않은 번호 또는 팀: {r}')
        slots.append(slot)
        teams.append(team)
    if len(slots) != len(set(slots)) or len(teams) != len(set(teams)):
        raise ValueError('그리드 또는 팀 중복')
    if sorted(slots) != list(range(1, len(slots)+1)):
        raise ValueError('그리드 번호는 1부터 연속이어야 합니다')
    if count is not None and len(rows) != count:
        raise ValueError(f'인원수 불일치: {len(rows)} != {count}')
    return {int(r['grid']): r['team'].strip() for r in rows}


def validate_poses(path, count=None):
    rows = load_csv(path, ('grid', 'x', 'y', 'road_z', 'yaw_rad'))
    slots = [int(r['grid']) for r in rows]
    if len(slots) != len(set(slots)) or sorted(slots) != list(range(1, len(slots)+1)):
        raise ValueError('그리드 번호 중복 또는 누락')
    if count is not None and len(rows) < count:
        raise ValueError('스폰 위치 부족')
    for row in rows:
        for name in ('x', 'y', 'road_z', 'yaw_rad'):
            value = float(row[name]); import math
            if not math.isfinite(value):
                raise ValueError('좌표에 NaN/Inf 사용 불가')
    # This checks only syntax. Track inclusion, collision, slope and spacing
    # MUST be checked against official geometry by race_manager before spawn.
    return rows


def draw_tracks(path, seed=None):
    rng = random.Random(seed)
    qualifying = rng.choice([x for x in TRACKS if x not in ('monaco','cota','baku')])
    remaining = [x for x in TRACKS if x != qualifying]
    # Track reports: cota ~1105m, baku ~1192m. Prefer this close pair.
    pair = ('cota', 'baku')
    if all(p in remaining for p in pair):
        a, b = rng.sample(pair, 2)
    else:
        lengths = {'monza': 1150., 'spa': 1399., 'monaco': 667., 'cota': 1105., 'hungaroring': 877., 'baku': 1192.}
        # Monza/Monaco figures here are NOT verified; prefer to abort rather than
        # use estimates for fairness. Override by choosing a COTA/BAKU compatible qualifier.
        raise ValueError('예선 선택으로 검증된 본선 근접 길이 맵(COTA/BAKU) 사용 불가: 다시 추첨하세요')
    finals = [x for x in TRACKS if x not in {qualifying, a, b, 'monza'}]
    if not finals:
        raise ValueError('결승 맵 후보 없음')
    payload = {'created_at':dt.datetime.now(dt.timezone.utc).isoformat(),
               'seed':seed, 'qualifying':qualifying, 'semi_a':a, 'semi_b':b,
               'final':rng.choice(finals), 'pool':list(TRACKS)}
    target=Path(path)
    if target.exists():
        raise ValueError('이미 추첨 결과가 존재합니다. 원본을 보존하고 새 경기를 만드세요.')
    target.parent.mkdir(parents=True,exist_ok=True)
    atomic_json(target,payload)
    return payload


def atomic_json(path, obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(prefix='.'+path.name,dir=path.parent)
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as f:
            json.dump(obj,f,ensure_ascii=False,indent=2);f.write('\n');f.flush();os.fsync(f.fileno())
        os.replace(name,path)
    finally:
        if os.path.exists(name):os.unlink(name)


def record(path, event):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    obj={'time_utc':dt.datetime.now(dt.timezone.utc).isoformat(), **event}
    with path.open('a',encoding='utf-8') as f:
        f.write(json.dumps(obj,ensure_ascii=False)+'\n');f.flush();os.fsync(f.fileno())


def gz_service(world, service, request_type, request, timeout=10):
    if not re.fullmatch(r'[a-z][a-z0-9_]*',world):raise ValueError('잘못된 월드 이름')
    cmd=['gz','service','-s',f'/world/{world}/{service}','--reqtype',request_type,
         '--reptype','gz.msgs.Boolean','--timeout',str(timeout*1000),'--req',request]
    result=subprocess.run(cmd,text=True,capture_output=True,timeout=timeout+5)
    if result.returncode or not re.search(r'\bdata:\s*true\b',result.stdout,re.I):
        raise RuntimeError(f'{service} 실패: {result.stdout[-500:]} {result.stderr[-500:]}')


def pause_world(world):
    gz_service(world,'control','gz.msgs.WorldControl','pause: true')


def resume_world(world):
    gz_service(world,'control','gz.msgs.WorldControl','pause: false')


def remove_entity(world,team):
    if not ENTRY_RE.fullmatch(team):raise ValueError('팀 ID 오류')
    gz_service(world,'remove','gz.msgs.Entity',f'name: "{team}" type: MODEL')


def delete_grid(world,grid,order,log,reason):
    if grid not in order:raise ValueError(f'존재하지 않는 그리드: {grid}')
    team=order[grid]
    # Important: paused state is retained on any error. Manual resume required.
    pause_world(world)
    record(log,{'action':'pause','world':world,'grid':grid,'team':team})
    try:
        remove_entity(world,team)
        record(log,{'action':'remove','world':world,'grid':grid,'team':team,
                    'status':'DNF','reason':reason})
    except Exception as exc:
        record(log,{'action':'remove_failed','world':world,'grid':grid,'team':team,'error':str(exc)})
        raise
    resume_world(world)
    record(log,{'action':'resume','world':world,'grid':grid,'team':team})
    return team


def main(argv=None):
    p=argparse.ArgumentParser(description='URRC 운영자 명령: 팀 제출물 생성/수정 없음')
    sp=p.add_subparsers(dest='command',required=True)
    check=sp.add_parser('check-grid');check.add_argument('--order',required=True);check.add_argument('--poses');check.add_argument('--count',type=int)
    draw=sp.add_parser('draw-maps');draw.add_argument('--output',required=True);draw.add_argument('--seed',type=int)
    pause=sp.add_parser('pause');pause.add_argument('--world',required=True)
    resume=sp.add_parser('resume');resume.add_argument('--world',required=True)
    delete=sp.add_parser('delete');delete.add_argument('--world',required=True);delete.add_argument('--grid',type=int,required=True)
    delete.add_argument('--order',required=True);delete.add_argument('--log',required=True)
    delete.add_argument('--reason',choices=['collision','blocking','manual'],required=True)
    a=p.parse_args(argv)
    try:
        if a.command=='check-grid':
            order=validate_order(a.order,a.count)
            if a.poses:validate_poses(a.poses,len(order))
            print(json.dumps(order,ensure_ascii=False,indent=2))
        elif a.command=='draw-maps':print(json.dumps(draw_tracks(a.output,a.seed),ensure_ascii=False,indent=2))
        elif a.command=='pause':pause_world(a.world);print('PAUSED')
        elif a.command=='resume':resume_world(a.world);print('RESUMED')
        elif a.command=='delete':print('REMOVED',delete_grid(a.world,a.grid,validate_order(a.order),a.log,a.reason))
        return 0
    except (ValueError,RuntimeError,subprocess.TimeoutExpired) as e:
        print('ERROR:',e,file=sys.stderr);return 2

if __name__=='__main__':sys.exit(main())
