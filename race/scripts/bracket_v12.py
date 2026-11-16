"""v12 competition field limits and small-field test bracket.

When only 2-3 qualifiers compete, all race in semi_a; semi_b is omitted.
With >=4 entrants, preserve odd/even qualifying-rank semi A/B allocation.
The 7th finalist (when available) is selected by qualifying rank among
semifinal 4th-place finishers; times on different circuits are incomparable.
"""
from pathlib import Path
import json

MIN_SEMI = 2
MAX_SEMI = 6
MIN_FINAL = 2
MAX_FINAL = 7
MAX_QUALIFYING = 12


def _read(path):
    path=Path(path)
    if not path.is_file():
        raise ValueError(f'이전 단계 결과가 없습니다: {path}')
    data=json.loads(path.read_text(encoding='utf-8'))
    if data.get('valid') is not True or data.get('state')!='complete' or data.get('provisional'):
        raise ValueError(f'이전 단계 확정 기록이 아닙니다: {path}')
    rows=data.get('results')
    if not isinstance(rows,list):
        raise ValueError(f'순위 데이터가 잘못되었습니다: {path}')
    return rows


def _ranks(rows, label):
    if len({r.get('entry_id') for r in rows})!=len(rows):
        raise ValueError(f'{label}: 중복 참가팀')
    if any(not isinstance(r.get('rank'),int) or isinstance(r.get('rank'),bool) for r in rows):
        raise ValueError(f'{label}: 순위 자료 오류')
    ranked=sorted(rows,key=lambda r:r['rank'])
    if [r['rank'] for r in ranked]!=list(range(1,len(rows)+1)) or any(r.get('rank_provisional') for r in rows):
        raise ValueError(f'{label}: 확정 순위가 연속·유일하지 않습니다.')
    return ranked


def eligible_ids(stage, results):
    """Return selected teams for a stage; [] means small-field semi_b skipped."""
    results=Path(results)
    if stage=='qualifying':
        return None
    q=_ranks(_read(results/'qualifying.json'),'예선')
    n=len(q)
    if not 2<=n<=MAX_QUALIFYING:
        raise ValueError(f'본선·결승 테스트는 예선 확정 결과 2~{MAX_QUALIFYING}팀이 필요합니다. 현재 {n}팀')
    qids=[r['entry_id'] for r in q]
    if stage in ('semi_a','semi_b'):
        if n<=3:
            if stage=='semi_b':return []
            return qids  # 2-3 teams: one actual semifinal, no fake/duplicate entrant
        ids=[r['entry_id'] for r in q if r['rank']%2==(1 if stage=='semi_a' else 0)]
        if not MIN_SEMI<=len(ids)<=MAX_SEMI:
            raise ValueError(f'{stage}: 본선 참가자는 {MIN_SEMI}~{MAX_SEMI}팀이어야 합니다.')
        return ids
    if stage!='final':
        raise ValueError(f'지원하지 않는 경기 단계: {stage}')

    legs=('semi_a',) if n<=3 else ('semi_a','semi_b')
    groups={}
    for leg in legs:
        expected=eligible_ids(leg,results)
        rows=_ranks(_read(results/f'{leg}.json'),leg)
        if len(rows)!=len(expected) or {r['entry_id'] for r in rows}!=set(expected):
            raise ValueError(f'{leg}: 예선 배정 팀과 확정 기록의 참가팀이 다릅니다.')
        if not MIN_SEMI<=len(rows)<=MAX_SEMI:
            raise ValueError(f'{leg}: 본선은 {MIN_SEMI}~{MAX_SEMI}팀이어야 합니다.')
        groups[leg]=[r for r in rows if r.get('status')=='finished']

    finalists=[]
    # Alternating ranks avoids direct comparison of different semifinals' times.
    for place in range(3):
        for leg in legs:
            if len(groups[leg])>place:
                finalists.append(groups[leg][place]['entry_id'])
    if len(finalists)<MIN_FINAL:
        raise ValueError(f'결승 출전 가능한 본선 완주자가 최소 {MIN_FINAL}팀 필요합니다.')
    # 7th place: fastest *qualifying rank*, not incompatible semi A/B race times.
    if len(finalists)==6 and len(legs)==2:
        candidate_ids=[groups[leg][3]['entry_id'] for leg in legs if len(groups[leg])>=4]
        qualifying_rank={row['entry_id']:row['rank'] for row in q}
        candidate_ids.sort(key=lambda eid:qualifying_rank[eid])
        finalists.extend(candidate_ids[:MAX_FINAL-len(finalists)])
    if len(finalists)>MAX_FINAL or len(set(finalists))!=len(finalists):
        raise ValueError('결승 참가자 제한 또는 중복 오류')
    return finalists
