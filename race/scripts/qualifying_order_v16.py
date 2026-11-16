"""Select only the *first* qualifier; preserve the complete sequential race."""
from __future__ import annotations
import os
import re


def ordered(entries, first=None):
    entries = sorted(entries, key=lambda e: e['entry_id'])
    first = os.environ.get('URRC_QUALIFYING_FIRST') if first is None else first
    if not first:
        return entries
    if not re.fullmatch(r'team_(?:0[1-9]|1[0-2])', first):
        raise ValueError(f'예선 시작 참가팀 형식 오류: {first} (team_01~team_12)')
    ids = [e['entry_id'] for e in entries]
    if first not in ids:
        raise ValueError(f'예선 시작 참가팀이 등록되어 있지 않습니다: {first}; 등록 팀={ids}')
    start = ids.index(first)
    result = entries[start:] + entries[:start]
    print('[v16 예선 순서] ' + ' → '.join(e['entry_id'] for e in result), flush=True)
    return result
