#!/usr/bin/env python3
"""URRC v3: refresh independent stage maps; allow empty-entrant map previews."""
import argparse
import os
import signal
import itertools
import json
import random
import subprocess
import sys
import time
from pathlib import Path

POOL=('monza','spa','monaco','cota','hungaroring','baku')
STAGES=('qualifying','semi_a','semi_b','final')
SEMI_PAIR=('cota','baku')  # surveyed 1105m/1192m; near-length pair

def assignments():
    for a,b in (SEMI_PAIR,SEMI_PAIR[::-1]):
        remaining=[m for m in POOL if m not in (a,b)]
        for q in remaining:
            if q=='monaco': continue
            for f in remaining:
                if f in ('monza',q): continue
                yield dict(zip(STAGES,(q,a,b,f)))

def refresh(path, rng=None):
    rng=rng or random.SystemRandom()
    previous=load(path) if path.exists() else None
    all_options=list(assignments())
    if previous:
        prev=previous['maps']
        changed=[o for o in all_options if all(o[s]!=prev.get(s) for s in STAGES)]
        if changed: all_options=changed
        else: all_options=[o for o in all_options if o!=prev] or all_options
    maps=rng.choice(all_options)
    data={'version':3,'updated_unix':time.time(),'maps':maps,
          'rules':'unique stages; qualifying != monaco; final != monza; cota/baku paired semifinals'}
    path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    temp.replace(path)
    return data

def load(path):
    data=json.loads(path.read_text(encoding='utf-8'))
    maps=data['maps']
    if set(maps)!=set(STAGES) or len(set(maps.values()))!=4:
        raise ValueError('맵 설정이 잘못됐습니다. refresh-maps를 실행하십시오.')
    if maps['qualifying']=='monaco' or maps['final']=='monza':
        raise ValueError('맵 제외 조건에 위배됩니다.')
    return data

def entrant_count(root):
    folder=root/'race/entrants'
    return sum(1 for p in folder.glob('*/race_entry.json') if p.is_file()) if folder.exists() else 0

def _v13_call(cmd,root):
    """Isolate child race from terminal SIGINT and let it clean itself up."""
    child=subprocess.Popen(cmd,cwd=root,start_new_session=True)
    try:
        return child.wait()
    except KeyboardInterrupt:
        try:os.killpg(child.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:child.wait(timeout=20)
        except subprocess.TimeoutExpired:
            try:os.killpg(child.pid,signal.SIGTERM)
            except ProcessLookupError:pass
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:os.killpg(child.pid,signal.SIGKILL)
                except ProcessLookupError:pass
                child.wait()
        print('[v13] 종료 요청 전달 완료',flush=True)
        return 130

def run_stage(root, stage, track, preview):
    print(f'[v3] {stage} -> {track} / 등록 차량 {entrant_count(root)}대',flush=True)
    if preview or entrant_count(root)==0:
        print('[v3] 맵만 실행합니다. 실제 차량 스폰·랩 계측·순위 저장은 수행하지 않습니다.',flush=True)
        cmd=['bash',str(root/'run_v10.sh'),track]
        if not (root/'run_v10.sh').is_file(): raise ValueError('run_v10.sh가 없습니다.')
        return _v13_call(cmd,root)
    # Existing v2 race timing logic; stage selection / participant checks stay v2's responsibility.
    cmd=['bash',str(root/'race_v2.sh'),stage,'--track',track]
    return _v13_call(cmd,root)


def main(argv=None):
    p=argparse.ArgumentParser(description='URRC F1 v3 independent maps and stage launcher')
    p.add_argument('command',choices=['refresh-maps','show-maps',*STAGES])
    p.add_argument('--root',default=str(Path(__file__).resolve().parents[2]))
    p.add_argument('--track',choices=POOL,help='이 실행에만 지정; 저장된 맵 배정은 변경하지 않음')
    p.add_argument('--preview',action='store_true',help='참가자가 있어도 맵만 표시')
    args=p.parse_args(argv)
    root=Path(args.root).expanduser().resolve()
    path=root/'race/results/v3_maps.json'
    try:
        if args.command=='refresh-maps':
            if args.track or args.preview: p.error('refresh-maps에는 --track/--preview를 사용하지 않습니다.')
            data=refresh(path)
        else:
            if not path.is_file():
                if args.command=='show-maps':raise ValueError('맵 배정이 없습니다. refresh-maps를 먼저 실행하십시오.')
                print('[v3] 최초 맵 배정을 생성합니다.',flush=True)
                refresh(path)
            data=load(path)
        if args.command in ('refresh-maps','show-maps'):
            print('맵 배정:',json.dumps(data['maps'],ensure_ascii=False,indent=2))
            print('설정 파일:',path)
            return 0
        track=args.track or data['maps'][args.command]
        if args.command=='qualifying' and track=='monaco':raise ValueError('예선 Monaco 제외')
        if args.command=='final' and track=='monza':raise ValueError('결승 Monza 제외')
        # v7: import only user-provided src/team/team_XX, without creating vehicles.
        imported=subprocess.run([sys.executable,str(root/'race/scripts/team_sync_v7.py'),
                                 'sync','--root',str(root)],check=False)
        if imported.returncode!=0:raise ValueError('src/team 자동 접수에 실패했습니다.')
        return run_stage(root,args.command,track,args.preview)
    except (ValueError,OSError,KeyError,json.JSONDecodeError) as e:
        print(f'[v3] 오류: {e}',file=sys.stderr);return 2
if __name__=='__main__':sys.exit(main())
