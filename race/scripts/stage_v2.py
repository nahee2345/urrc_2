#!/usr/bin/env python3
"""Independent race-stage launcher around the repository's official race manager.
Does not alter entrant packages or world assets.
"""
import argparse
import csv
import inspect
import json
import random
import re
import sys
import uuid
from pathlib import Path

POOL=('monza','spa','monaco','cota','hungaroring','baku')

def choose_track(stage, requested, exclude, rng):
    forbidden=set(exclude)
    if stage=='qualifying': forbidden.add('monaco')
    if stage=='final': forbidden.add('monza')
    if requested:
        if requested not in POOL or requested in forbidden:
            raise ValueError(f'{stage}: 허용되지 않는 맵 {requested}; 제외={sorted(forbidden)}')
        return requested
    available=[x for x in POOL if x not in forbidden]
    if not available: raise ValueError('선택 가능한 맵이 없습니다.')
    return rng.choice(available)

def verified_rows(path):
    if not path.is_file(): raise ValueError(f'이전 단계 결과가 없습니다: {path}')
    data=json.loads(path.read_text(encoding='utf-8'))
    if data.get('valid') is not True or data.get('state')!='complete' or data.get('provisional'):
        raise ValueError(f'이전 단계의 확정 기록이 아닙니다: {path}')
    return data['results']

def eligible_ids(stage, results):
    # v12: delegate only bracket logic; retain original runner/timing/lighting.
    from bracket_v12 import eligible_ids as select
    return select(stage, results)


def export_fast_lap(results):
    rows=verified_rows(results/'qualifying.json')
    target=results.parent.parent/'src'/'fast_lap.txt'
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.open('w',encoding='utf-8',newline='') as f:
        writer=csv.writer(f);writer.writerow(['rank','team','total_seconds','status'])
        for row in sorted(rows,key=lambda r:r['rank']):
            writer.writerow([row['rank'],row['entry_id'],'' if row.get('time_seconds') is None else f"{row['time_seconds']:.6f}",row['status']])
    print(f'예선 기록 저장: {target}')

def run(args):
    root=Path(args.root).expanduser().resolve()
    sys.path.insert(0,str(root/'race/scripts'))
    import race_manager as mgr
    stage=args.stage
    results=root/'race/results';results.mkdir(parents=True,exist_ok=True)
    track=choose_track(stage,args.track,args.exclude,random.SystemRandom())
    desired=eligible_ids(stage,results)
    if stage=='semi_b' and desired==[]:
        print('[v12] 예선 2~3팀: 본선 A에 전원 출전, 본선 B는 생략합니다.', flush=True)
        return
    registered_count=sum(1 for p in (root/'race/entrants').glob('*/race_entry.json') if p.is_file())
    expected=registered_count if stage=='qualifying' else len(desired)
    original_validate=mgr.validate
    original_save=mgr.save_results
    original_get=mgr.get_event
    def selected_validate(r):
        all_entries=original_validate(r)
        if not 1<=len(all_entries)<=12:raise ValueError(f'등록 참가자는 1~12팀이어야 합니다. 현재 {len(all_entries)}팀')
        if stage!='qualifying' and len(all_entries)<2:raise ValueError('본선·결승에는 최소 2팀의 예선 등록이 필요합니다.')
        if len(all_entries)!=registered_count:raise ValueError('검수 중 참가자 수가 변경됐습니다.')
        by_id={e['entry_id']:e for e in all_entries}
        if stage=='qualifying':
            from qualifying_order_v16 import ordered
            return ordered(all_entries)
        if desired is not None:
            if len(desired)!=expected or any(e not in by_id for e in desired):
                raise ValueError('기록에 있는 참가팀이 제출 목록과 일치하지 않습니다.')
            selected=[by_id[e] for e in desired]
        else:
            selected=sorted(all_entries,key=lambda e:e['entry_id'])
        # v8: check genuine user-supplied geometry (no test cars are generated).
        from base_link_check_v6 import check as v8_base_check
        import os
        for entry in selected:
            report=v8_base_check(entry['_model'],entry.get('base_link_name','base_link'))
            if report['status']=='FAIL':
                raise ValueError(f"{entry['entry_id']}: base_link geometry FAIL: {report['errors']}")
            if report['status']=='REVIEW_REQUIRED':
                print(f"[v8] {entry['entry_id']}: base_link manual review required: {report['warnings']}",flush=True)
                if os.environ.get('URRC_BASE_REVIEW_APPROVED')!='1':
                    raise ValueError('base_link manual review required; verify geometry, then set URRC_BASE_REVIEW_APPROVED=1')
        # v6 spectator-only metadata. Never create or alter participant files.
        grid_file=results/'active_grid_v6.json'
        grid_file.write_text(json.dumps({'stage':stage,'track':track,
                             'teams':[e['entry_id'] for e in selected]},ensure_ascii=False,indent=2)+'\n')
        return selected
    event={'event_id':uuid.uuid4().hex[:12], 'track':track,'seed':random.SystemRandom().randrange(2**32)}
    def stage_save(res,run_dir,mode,payload):
        payload['v2_stage']=stage
        return original_save(res,run_dir,stage,payload)
    # Reuse all official timing/spawn/start/light/cleanup code. Remove only the
    # obsolete 'final requires same-track qualifying' guard from its function.
    source=inspect.getsource(mgr.run_mode)
    from host_guard_v8 import HostGuard
    health=HostGuard(results/'host_errors_v8.jsonl').check
    # The source is compiled below by v2; injection is scoped only to official race processes.
    if source.count('        monitor.check()')!=1:
        raise RuntimeError('v8: check_world() health hook not found')
    source=source.replace('        monitor.check()',
                          '        monitor.check()\n        v8_health()',1)
    # The vehicle must be spawned and its controller ready before the 3-second signal delay.
    old='            start_sequence(track,root,active,'
    if source.count(old)!=1:
        raise RuntimeError('v8: starting sequence hook not found')
    source=source.replace(old,
        '            gate.publish([e[\"entry_id\"] for e in active],False)\n'
        '            time.sleep(3)\n'
        '            check_world()\n'+old,1)

    old="    if mode=='final':\n        qpath=results/'qualifying.json'"
    if source.count(old)!=1:
        raise RuntimeError('기존 race_manager.py 버전이 변경됐습니다. 임의로 실행하지 않습니다.')
    source=source.replace(old,"    if False:  # v2: independent stages\n        qpath=results/'qualifying.json'",1)
    # The older final mode rejects 3-lap runs; semifinals require exactly 3.
    old_lap_guard="    if mode=='final' and not 5<=laps<=10:"
    if source.count(old_lap_guard)!=1: raise RuntimeError('기존 랩 제한 코드가 예상과 다릅니다.')
    source=source.replace(old_lap_guard,"    if mode=='final' and not (3<=laps<=10):",1)
    # Distinct run directories for stages while retaining the original audit output.
    source=source.replace('f"{mode}_{uuid.uuid4().hex[:12]}"','f"{stage}_{uuid.uuid4().hex[:12]}"')
    scope=dict(mgr.__dict__);scope.update(validate=selected_validate,save_results=stage_save,get_event=lambda *a,**k:event,stage=stage,v8_health=health)
    exec(compile(source,str(root/'race/scripts/race_manager.py'),'exec'),scope)
    internal_mode='qualifying' if stage=='qualifying' else 'final'
    laps=2 if stage=='qualifying' else 3 if stage.startswith('semi_') else 5
    print(f'[v12] 본선 2~6대 / 결승 2~7대 / {stage} / {track} / {expected}대 / {laps}랩 / 카메라 신호 출발')
    scope['run_mode'](root,internal_mode,None if stage=='qualifying' else laps,'camera')
    # Semifinal stages are intentionally independent; no event-wide map lock.
    if stage=='qualifying':export_fast_lap(results)
    print(f'[v2] 완료. 기록: {results/stage}.json')

def main():
    p=argparse.ArgumentParser(description='URRC F1 independent stage v2')
    p.add_argument('stage',choices=['qualifying','semi_a','semi_b','final'])
    p.add_argument('--root',default=str(Path(__file__).resolve().parents[2]))
    p.add_argument('--track',choices=POOL,help='생략 시 해당 단계에서 랜덤 추첨')
    p.add_argument('--exclude',action='append',choices=POOL,default=[],help='랜덤 추첨에서 제외할 맵. 여러 번 지정 가능')
    a=p.parse_args()
    try:run(a)
    except KeyboardInterrupt:
        print('[v13] 운영자 중단(SIGINT)',file=sys.stderr)
        return 130
    except Exception as e:print(f'[v2] 오류: {e}',file=sys.stderr);return 2
    return 0

if __name__=='__main__':sys.exit(main())
