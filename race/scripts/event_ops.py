#!/usr/bin/env python3
"""Operator event state, audit, DNF and replay manifest. Does not alter entrants."""
import argparse, datetime as dt, hashlib, json, os, re, sys, tempfile
from pathlib import Path

STAGES=('qualifying','semi_a','semi_b','final')
TEAM=re.compile(r'^team_(0[1-9]|10)$')

def atomic(path,obj):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(prefix='.'+path.name,dir=path.parent)
    try:
        with os.fdopen(fd,'w',encoding='utf-8') as f:
            json.dump(obj,f,indent=2,ensure_ascii=False);f.write('\n');f.flush();os.fsync(f.fileno())
        os.replace(tmp,path)
    finally:
        if os.path.exists(tmp):os.unlink(tmp)

def read(path):return json.loads(Path(path).read_text(encoding='utf-8'))
def stamp():return dt.datetime.now(dt.timezone.utc).isoformat()
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def initiate(folder,draw,stage,order,poses=None):
    from admin_control import validate_order,validate_poses
    folder=Path(folder)
    if folder.exists():raise ValueError('이미 세션이 존재합니다. 새 디렉터리를 선택하세요.')
    d=read(draw)
    if stage not in STAGES:raise ValueError('단계 오류')
    track=d[stage]
    slots=validate_order(order)
    if stage=='qualifying' and len(slots)!=10:raise ValueError('예선은 10대 그리드가 필요합니다')
    if stage in ('semi_a','semi_b') and len(slots)!=5:raise ValueError('본선은 5대 그리드가 필요합니다')
    if stage=='final' and len(slots)!=6:raise ValueError('결승은 6대 그리드가 필요합니다')
    if poses:validate_poses(poses,len(slots))
    stages_laps={'qualifying':2,'semi_a':3,'semi_b':3,'final':5}
    obj=dict(schema=1,stage=stage,track=track,laps=stages_laps[stage],status='prepared',created_at=stamp(),
             draw_sha256=digest(draw),draw_file=str(Path(draw).resolve()),
             order_sha256=digest(order),order_file=str(Path(order).resolve()),
             poses_sha256=digest(poses) if poses else None,poses_file=str(Path(poses).resolve()) if poses else None,
             grid={str(k):v for k,v in slots.items()},dnf={},events=[],invalid_reason=None)
    folder.mkdir(parents=True)
    atomic(folder/'session.json',obj)
    return obj

def transition(folder,action,grid=None,reason=None):
    folder=Path(folder);path=folder/'session.json';o=read(path)
    if action=='dnf':
        if o['status'] not in ('prepared','running','paused'):raise ValueError('DNF 처리 불가능 상태')
        team=o['grid'].get(str(grid))
        if not team:raise ValueError('존재하지 않는 그리드')
        if str(grid) in o['dnf']:raise ValueError('이미 DNF 처리됨')
        if not reason:raise ValueError('DNF 사유 필요')
        o['dnf'][str(grid)]=dict(team=team,reason=reason,at=stamp())
    elif action=='invalidate':
        if not reason:raise ValueError('경기 무효 사유 필요')
        o['status']='invalid';o['invalid_reason']=reason
    elif action=='start':
        if o['status']!='prepared':raise ValueError('시작할 수 없는 상태');
        o['status']='running'
    elif action=='pause':
        if o['status']!='running':raise ValueError('일시정지할 수 없는 상태')
        o['status']='paused'
    elif action=='resume':
        if o['status']!='paused':raise ValueError('재개할 수 없는 상태')
        o['status']='running'
    elif action=='finish':
        if o['status'] not in ('running','paused'):raise ValueError('종료할 수 없는 상태')
        o['status']='finished'
    else:raise ValueError('알 수 없는 작업')
    o['events'].append(dict(action=action,grid=grid,reason=reason,at=stamp()))
    atomic(path,o);return o

def replay(original,new):
    source=read(Path(original)/'session.json')
    if source['status']!='invalid':raise ValueError('무효 처리된 세션만 재경기 가능')
    if Path(new).exists():raise ValueError('새 재경기 디렉터리가 이미 존재')
    for key,sha in [('draw_file','draw_sha256'),('order_file','order_sha256'),('poses_file','poses_sha256')]:
        path=source.get(key)
        if path and digest(path)!=source[sha]:raise ValueError(f'원본 설정 파일 변경 감지: {path}')
    newobj={**source,'status':'prepared','created_at':stamp(),'dnf':{},'events':[],
            'invalid_reason':None,'replay_of':str(Path(original).resolve())}
    Path(new).mkdir(parents=True)
    atomic(Path(new)/'session.json',newobj)
    return newobj

def export_fast(session,results,dest):
    """Extract verified qualifying results JSON from existing race_manager output.
    Reject invalid or provisional result records. Never infer absent rows.
    """
    o=read(Path(session)/'session.json')
    if o['stage']!='qualifying':raise ValueError('예선에서만 생성 가능')
    if o['status']=='invalid':raise ValueError('무효 세션')
    data=read(results)
    if data.get('valid') is False:raise ValueError('무효 계측 결과')
    rows=data.get('results')
    if not isinstance(rows,list) or len(rows)!=10:raise ValueError('예선 기록은 10명이어야 합니다')
    seen=set();out=[]
    for r in rows:
        team=r.get('entry_id')
        if team not in o['grid'].values() or team in seen:raise ValueError('참가자 불일치 또는 중복')
        seen.add(team)
        if r.get('rank_provisional'):raise ValueError('잠정 순위가 있습니다. 재계측이 필요합니다')
        status=r.get('status')
        if status not in ('finished','DNF'):raise ValueError('유효하지 않은 상태')
        t=r.get('time_seconds')
        if status=='finished' and (not isinstance(t,(int,float)) or t<=0):raise ValueError('완주 기록 오류')
        out.append((r.get('rank'),team,t,status))
    if sorted(r[0] for r in out)!=list(range(1,11)):raise ValueError('순위 누락/중복')
    out.sort()
    p=Path(dest);p.parent.mkdir(parents=True,exist_ok=True)
    if p.exists():raise ValueError('기존 결과 덮어쓰기 금지')
    contents='# rank,team,total_seconds,status\n'+''.join(f'{rk},{team},{"" if tm is None else f"{tm:.3f}"},{status}\n' for rk,team,tm,status in out)
    p.write_text(contents,encoding='utf-8')
    return str(p)

def main(argv=None):
    p=argparse.ArgumentParser();s=p.add_subparsers(dest='cmd',required=True)
    a=s.add_parser('init');a.add_argument('--session',required=True);a.add_argument('--draw',required=True);a.add_argument('--stage',choices=STAGES,required=True);a.add_argument('--order',required=True);a.add_argument('--poses')
    a=s.add_parser('transition');a.add_argument('--session',required=True);a.add_argument('--action',choices=['start','pause','resume','finish','invalidate','dnf'],required=True);a.add_argument('--grid',type=int);a.add_argument('--reason')
    a=s.add_parser('replay');a.add_argument('--original',required=True);a.add_argument('--new',required=True)
    a=s.add_parser('status');a.add_argument('--session',required=True)
    a=s.add_parser('export-fast');a.add_argument('--session',required=True);a.add_argument('--results',required=True);a.add_argument('--output',required=True)
    x=p.parse_args(argv)
    try:
        if x.cmd=='init':o=initiate(x.session,x.draw,x.stage,x.order,x.poses)
        elif x.cmd=='transition':o=transition(x.session,x.action,x.grid,x.reason)
        elif x.cmd=='replay':o=replay(x.original,x.new)
        elif x.cmd=='status':o=read(Path(x.session)/'session.json')
        elif x.cmd=='export-fast':print(export_fast(x.session,x.results,x.output));return 0
        print(json.dumps(o,indent=2,ensure_ascii=False));return 0
    except (ValueError,FileNotFoundError,KeyError) as e:
        print('ERROR:',e,file=sys.stderr);return 2
if __name__=='__main__':sys.exit(main())
