#!/usr/bin/env python3
"""URRC v18: read-only compact TXT snapshots of official and rehearsal results.

Writes ONLY race/results/summary_txt/*.txt. Reads previous official stage
results, operator preflight reports, and independent rehearsal reports.
No scoring, qualifying, participant, map, or official results are changed.
"""
from __future__ import annotations
import argparse
import datetime as dt
import json
import os
from pathlib import Path
import re
import sys

STAGES = ('vehicles','qualifying','semi_a','semi_b','final')
LABEL = {'vehicles':'차량검수', 'qualifying':'예선', 'semi_a':'본선A',
         'semi_b':'본선B', 'final':'결승'}
TEAM_RE = re.compile(r'^team_(?:0[1-9]|1[0-2])$')
LAPS = {'qualifying':2,'semi_a':3,'semi_b':3,'final':5}
RESULT_DIR = Path('race/results/summary_txt')
INSPECTION_DIR = Path('race/results/inspections')


def read_json(path):
    try:
        value=json.loads(path.read_text(encoding='utf-8'))
        if isinstance(value, dict):
            return value
    except (OSError, UnicodeError, ValueError):
        pass
    return None


def compact(value, maximum=72):
    text=' '.join(str(value or '').split()).replace('|', '/')
    return text if len(text)<=maximum else text[:maximum-1]+'…'


def timestamp(path):
    try:
        return dt.datetime.fromtimestamp(path.stat().st_mtime).strftime('%m-%d %H:%M')
    except OSError:
        return '-'


def duration(value):
    try:
        t=float(value)
        if t<0 or not (t<float('inf')):return '-'
        minutes=int(t//60)
        return f'{minutes:02d}:{t-minutes*60:06.3f}'
    except (ValueError, TypeError, OverflowError):
        return '-'


def stage_state(payload):
    if payload.get('valid') is False or payload.get('state')=='invalid':
        return 'INVALID'
    if payload.get('state')=='running':return 'RUNNING'
    if payload.get('provisional'):return 'REVIEW'
    if payload.get('state')=='complete' and payload.get('valid') is True:
        return 'COMPLETE'
    return 'UNKNOWN'


def team_sort(team):
    if TEAM_RE.fullmatch(team):return int(team[-2:])
    return 99


def official_rows(root):
    rows={}
    for stage in STAGES[1:]:
        path=root/'race/results'/f'{stage}.json'
        payload=read_json(path)
        rows[stage]=(payload,path)
    return rows


def preflight_rows(root):
    # Persistent cache from v18 wrappers. Per-team reports override all-team
    # reports only if recorded later. Legacy direct v16 runs with --report can
    # still be imported by placing that report in this directory.
    base=root/INSPECTION_DIR
    found={}
    if base.is_dir():
        for path in sorted(base.glob('current_*.json'),key=lambda p:(p.stat().st_mtime,p.name)):
            payload=read_json(path)
            if not payload: continue
            for item in payload.get('teams',[]):
                if not isinstance(item,dict):continue
                team=item.get('team')
                if not isinstance(team,str) or not TEAM_RE.fullmatch(team):continue
                if item.get('status') in ('EMPTY','MISSING'):continue
                found[team]={'status':item.get('status','UNKNOWN'),
                             'errors':item.get('errors') or [],
                             'warnings':item.get('warnings') or [],
                             'live':payload.get('live') if payload.get('registered')==1 else None,
                             'source_time':timestamp(path)}
    # No paid cost or Gazebo operation: fill unreported teams from original
    # metadata only as '미검수' rather than claiming static PASS.
    base=root/'src/team'
    if base.is_dir():
        for path in sorted(base.glob('team_*/race_entry.json')):
            team=path.parent.name
            if TEAM_RE.fullmatch(team) and team not in found:
                found[team]={'status':'미검수','errors':[], 'warnings':[], 'live':None,
                             'source_time':'-'}
    return found


def latest_rehearsals(root):
    base=root/'race/previews'
    data={}
    if not base.is_dir():return data
    for path in sorted(base.glob('v17_*/report.json'),key=lambda p:(p.stat().st_mtime,p.name)):
        item=read_json(path)
        if not item or item.get('mode')!='REHEARSAL_NOT_OFFICIAL':continue
        stage=item.get('stage')
        if stage not in STAGES:continue
        for eid, row in item.get('details',{}).items():
            if TEAM_RE.fullmatch(str(eid)) and isinstance(row,dict):
                data[(stage,eid)]=(item,row,path)
    return data


def known_teams(root,official,inspections,rehearsals):
    teams=set(inspections)
    for payload,_ in official.values():
        if payload:
            teams.update(str(r.get('entry_id')) for r in payload.get('results',[]) if isinstance(r,dict))
    teams.update(eid for _,eid in rehearsals)
    return sorted((t for t in teams if TEAM_RE.fullmatch(t)),key=team_sort)


def _official_line(row, stage, state):
    rank=row.get('rank')
    place=f'{rank}위' if rank not in (None,'') else '-'
    completed = row.get('status')=='finished' and (row.get('laps') or 0)>=LAPS[stage]
    if state=='COMPLETE':
        status='PASS' if completed else 'DNF'
    elif state=='INVALID':
        status='INVALID(개별 완주 무효)' if completed else 'INVALID(DNF)'
    else:
        status=f'{state}(개별 완주 미확정)' if completed else f'{state}(DNF)'
    lap=row.get('laps',0)
    elapsed=duration(row.get('time_seconds'))
    detail='' if status.startswith('PASS') else compact(row.get('reason'),64)
    result=f'{place} | {status} | {lap}/{LAPS[stage]}랩 | {elapsed}'
    if detail:result+=' | '+detail
    return result


def _practice_line(row, stage, report):
    status=row.get('status','UNKNOWN')
    if report.get('status') not in ('OBSERVATION_COMPLETE',) and status=='PASS':
        status+='(전체 연습 '+str(report.get('status'))+')'
    lap=row.get('laps',0)
    detail=compact(row.get('reason'),60)
    result=f'{status} | {lap}/{LAPS.get(stage,2)}랩'
    if detail:result+=' | '+detail
    return result


def emit_stage(stage, official, inspections, rehearsals, teams):
    lines=[f'[{LABEL[stage]}]']
    if stage=='vehicles':
        if not inspections:
            lines.append('등록 또는 검수 기록 없음')
        else:
            for eid in teams:
                record=inspections.get(eid)
                if not record:continue
                status=record.get('status','미검수')
                details=record.get('errors') or record.get('warnings') or []
                desc=' | '+compact(details[0],80) if details else ''
                if len(details)>1:desc+=f' (+{len(details)-1})'
                live=record.get('live')
                live_label=f' | 통신 {live.get("status")}' if isinstance(live,dict) else ''
                lines.append(f'{eid} | {status}{live_label}{desc}')
        runs=[(eid,*rehearsals[(stage,eid)]) for eid in teams if (stage,eid) in rehearsals]
        if runs:
            lines.append('--- 최신 모의점검 (공식 검수 아님) ---')
            for eid,report,row,path in runs:
                lines.append(f'{eid} | {row.get("status","UNKNOWN")} | 이동={row.get("max_displacement_m","-")}m | {timestamp(path)}')
        return '\n'.join(lines)+'\n'
    payload,path=official[stage]
    if not payload:
        lines.append('공식 경기: 미실행 / 결과 없음')
    else:
        state=stage_state(payload)
        lines.append(f'공식 경기 | {payload.get("track","-")} | {state} | 기록 {timestamp(path)}')
        stage_rows=[r for r in payload.get('results',[]) if isinstance(r,dict) and TEAM_RE.fullmatch(str(r.get('entry_id','')))]
        stage_rows.sort(key=lambda row:(row.get('rank') if isinstance(row.get('rank'),int) else 99,team_sort(row['entry_id'])))
        for r in stage_rows:
            lines.append(f'{r["entry_id"]} | {_official_line(r,stage,state)}')
        if not stage_rows:lines.append('기록된 차량 없음')
        if payload.get('error'):lines.append('경기 오류 | '+compact(payload.get('error')))
    runs=[(eid,*rehearsals[(stage,eid)]) for eid in teams if (stage,eid) in rehearsals]
    if runs:
        lines.append('--- 최신 모의주행 (비공식) ---')
        for eid,report,row,path in runs:
            lines.append(f'{eid} | {_practice_line(row,stage,report)} | {timestamp(path)}')
    return '\n'.join(lines)+'\n'


def emit_team(eid,official,inspections,rehearsals):
    lines=[eid]
    v=inspections.get(eid)
    if v:
        details=v.get('errors') or v.get('warnings') or []
        text=v['status']
        if details:text+=' | '+compact(details[0],64)
        lines.append('차량검수 | '+text)
    else:lines.append('차량검수 | 기록 없음')
    for stage in STAGES[1:]:
        payload,path=official[stage]
        row=next((r for r in payload.get('results',[]) if isinstance(r,dict) and r.get('entry_id')==eid),None) if payload else None
        if row:
            result=_official_line(row,stage,stage_state(payload))
            lines.append(f'{LABEL[stage]} | {result}')
        else:lines.append(f'{LABEL[stage]} | 공식 기록 없음')
        if (stage,eid) in rehearsals:
            report,rehearsal,path=rehearsals[(stage,eid)]
            lines.append(f'  연습 | {_practice_line(rehearsal,stage,report)}')
    return '\n'.join(lines)+'\n'


def write_atomic(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.v18.tmp')
    try:
        tmp.write_text(data,encoding='utf-8')
        os.replace(tmp,path)
    finally:
        tmp.unlink(missing_ok=True)


def generate(root):
    root=root.expanduser().resolve()
    if not (root/'race/scripts/race_manager.py').is_file():
        raise ValueError('URRC 대회 운영 폴더가 아닙니다: '+str(root))
    official=official_rows(root)
    inspections=preflight_rows(root)
    rehearsals=latest_rehearsals(root)
    teams=known_teams(root,official,inspections,rehearsals)
    output=root/RESULT_DIR
    output.mkdir(parents=True,exist_ok=True)
    for stage in STAGES:
        write_atomic(output/(stage+'.txt'),emit_stage(stage,official,inspections,rehearsals,teams))
    for eid in teams:
        write_atomic(output/(eid+'.txt'),emit_team(eid,official,inspections,rehearsals))
    lines=['[URRC F1 결과 요약]','공식 기록과 연습 기록은 별도 표시합니다.']
    for stage in STAGES:
        if stage=='vehicles':
            checked=sum(1 for x in inspections.values() if x['status']!='미검수')
            failed=sum(1 for x in inspections.values() if x['status']=='FAIL')
            lines.append(f'{LABEL[stage]} | 검수 {checked}/{len(inspections)}팀 | FAIL {failed}팀')
        else:
            payload,_=official[stage]
            if payload:
                done=sum(1 for x in payload.get('results',[]) if x.get('status')=='finished')
                lines.append(f'{LABEL[stage]} | {stage_state(payload)} | 완주 {done}팀 | {payload.get("track","-")}')
            else:lines.append(f'{LABEL[stage]} | 공식 미실행')
    lines.append('팀별 상세: team_01.txt~team_12.txt (기록이 있는 팀만 생성)')
    write_atomic(output/'summary.txt','\n'.join(lines)+'\n')
    print('[v18] TXT 결과 갱신:',output)
    return output


def main(argv=None):
    p=argparse.ArgumentParser(description='URRC F1 차량검수·경기·모의주행 초간단 TXT 결과')
    p.add_argument('--root',type=Path,default=Path.home()/'urrc_f1_ws')
    a=p.parse_args(argv)
    try:generate(a.root)
    except (ValueError,OSError) as exc:
        print('[v18] TXT 생성 실패:',exc,file=sys.stderr)
        return 2
    return 0

if __name__=='__main__':sys.exit(main())
