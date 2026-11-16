#!/usr/bin/env python3
"""Read-only, live checkpoint-rank monitor. No participant ROS subscriptions."""
import argparse, json, math, time
from pathlib import Path

STAGES=('qualifying','semi_a','semi_b','final')

def load_json(path, fallback=None):
    try:return json.loads(Path(path).read_text(encoding='utf-8'))
    except (FileNotFoundError,ValueError,OSError):return fallback

def summarize(events, grid, track_length_m, active_results=None):
    """rank by completed laps and last checkpoint; gap is checkpoint-resolution estimate."""
    state={team:{'grid':i+1,'team':team,'laps':0,'gate':0,'completed':False,'started':None,'now':None,'status':'WAITING','crossings':0}
           for i,team in enumerate(grid)}
    for ev in events:
        team=ev.get('entry_id')
        if team not in state:continue
        s=state[team]
        if ev.get('type')=='arm':
            s.update(status='READY',started=None,laps=0,gate=0,crossings=0)
        if ev.get('type')=='pose':
            sample=ev.get('sample') or []
            if sample:s['now']=sample[0]
        if ev.get('type')=='crossing':
            gate=ev.get('gate')
            if not isinstance(gate,int):continue
            s['gate']=gate;s['crossings']+=1
            if gate==0:
                if s['started'] is None:s['started']=ev['time']
                else:s['laps']+=1
            s['status']='RUNNING'
    for r in active_results or []:
        team=r.get('entry_id')
        if team not in state:continue
        s=state[team];s['laps']=r.get('laps',s['laps'])
        s['status']='FINISHED' if r.get('status')=='finished' else 'DNF'
    # Gate number is not a physical meter coordinate. It is a monotonic progress surrogate.
    rows=list(state.values())
    for s in rows:
        s['elapsed_s']=round(max(0.,s['now']-s['started']),3) if s['now'] is not None and s['started'] is not None else None
        s['_progress']=s['crossings']
    rows.sort(key=lambda s:(s['status'] not in ('RUNNING','FINISHED'),-s['_progress'],s['status']!='FINISHED',s['grid']))
    leader=next((s for s in rows if s['status'] in ('RUNNING','FINISHED')),None)
    lead_progress=leader['_progress'] if leader else None
    for idx,s in enumerate(rows,1):
        s['rank']=idx
        s['gap_checkpoints']=max(0,lead_progress-s['_progress']) if lead_progress is not None and s['started'] is not None else None
        s.pop('_progress')
    return rows

def format_seconds(value):
    if value is None:return '--:--.---'
    m,s=divmod(value,60)
    return f'{int(m):02d}:{s:06.3f}'

def render(rows, stage, track):
    print('\033[2J\033[H',end='')
    print(f'URRC F1 LIVE | {stage} | {track} | 계측 시작: 첫 base_link 출발선 통과')
    print('순위 | 그리드 | 팀       | 랩수 | 총주행시간 | 상태       | 선두와의 차이')
    print('-'*85)
    for r in rows:
        gap='--' if r['gap_checkpoints'] is None else f"약 {r['gap_checkpoints']} 체크포인트"
        print(f"{r['rank']:>4} | {r['grid']:>6} | {r['team']:<8} | {r['laps']:>4} | {format_seconds(r['elapsed_s']):>10} | {r['status']:<10} | {gap}")
    print('\n참고: 선두와의 차이는 체크포인트 개수 차이이며 정확한 차량 간 거리(m)가 아닙니다. Ctrl+C 종료')

def follow_jsonl(path, offset, pending):
    if not path.is_file():return [],offset,pending
    with path.open('rb') as f:
        f.seek(0,2);size=f.tell()
        if size<offset:offset=0;pending=b''
        f.seek(offset);new=f.read();offset=f.tell()
    parts=(pending+new).split(b'\n');pending=parts.pop()
    out=[]
    for ln in parts:
        try:out.append(json.loads(ln))
        except (ValueError,UnicodeDecodeError):continue
    return out,offset,pending

def monitor(root,stage,interval,once=False):
    results=root/'race/results'
    if not (results/'active_grid_v6.json').exists():
        raise ValueError('진행 중 경기 그리드 정보가 없습니다. v6 설치 후 실제 경기를 실행하십시오.')
    grid_data=load_json(results/'active_grid_v6.json') or {}
    if grid_data.get('stage')!=stage:raise ValueError('실행 중인 경기 단계와 요청한 단계가 다릅니다.')
    path=results/stage+'.json'
    offset=0;pending=b'';events=[];last_pose={};actual=None
    while True:
        current=load_json(path,{})
        audit=current.get('audit_directory')
        if audit and audit!=actual:
            actual=audit;events=[];last_pose={};offset=0;pending=b''
        if actual:
            read,offset,pending=follow_jsonl(root/actual/'timing.jsonl',offset,pending)
            for ev in read:
                if ev.get('type')=='pose':last_pose[ev.get('entry_id')]=ev
                elif ev.get('type') in ('arm','crossing'):events.append(ev)
        rows=summarize(events+list(last_pose.values()),grid_data['teams'],0,current.get('results',[]))
        render(rows,stage,grid_data.get('track','?'))
        if once or current.get('state') in ('complete','invalid'):break
        time.sleep(interval)

def main(argv=None):
    p=argparse.ArgumentParser()
    p.add_argument('stage',choices=STAGES);p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2]);p.add_argument('--interval',type=float,default=1.);p.add_argument('--once',action='store_true')
    a=p.parse_args(argv)
    try:monitor(a.root.resolve(),a.stage,max(.25,a.interval),a.once)
    except (OSError,ValueError,KeyError) as e:print(f'모니터 오류: {e}');return 2
    except KeyboardInterrupt:pass
    return 0
if __name__=='__main__':raise SystemExit(main())
