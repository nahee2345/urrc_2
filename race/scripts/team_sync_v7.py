#!/usr/bin/env python3
"""Sync externally supplied entrants from src/team into race/entrants.

No sample teams or algorithms are generated. Does not mutate submitted sources.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile

TEAM = re.compile(r'team_(0[1-9]|1[0-2])\Z')
MARKER = '.urrc_v7_managed'
SKIP = {'build','install','log','.git','__pycache__','.pytest_cache'}


def files_digest(folder: Path) -> str:
    h=hashlib.sha256()
    for p in sorted(folder.rglob('*')):
        if p.is_symlink():
            raise ValueError(f'심볼릭 링크 제출 금지: {p}')
        if not p.is_file() or any(x in SKIP for x in p.relative_to(folder).parts):
            continue
        h.update(p.relative_to(folder).as_posix().encode()+b'\0')
        h.update(p.read_bytes()); h.update(b'\0')
    return h.hexdigest()


def metadata(folder: Path, team_id: str) -> dict:
    manifests = [folder/'race_entry.json'] if (folder/'race_entry.json').is_file() else []
    if not manifests and (folder/'src').is_dir():
        manifests = list((folder/'src').glob('*/race_entry.json'))
    if not manifests:
        manifests = list(folder.glob('*/race_entry.json'))
    if len(manifests)!=1:
        raise ValueError(f'{folder}: race_entry.json 정확히 1개가 필요합니다. (팀 루트 또는 대표 ROS 패키지)')
    data=json.loads(manifests[0].read_text(encoding='utf-8'))
    if data.get('entry_id')!=team_id:
        raise ValueError(f'{folder}: race_entry.json entry_id는 {team_id}이어야 합니다.')
    return data


def sync(root: Path, *, dry_run=False) -> list:
    source=root/'src'/'team'
    target=root/'race'/'entrants'
    source.mkdir(parents=True,exist_ok=True)
    target.mkdir(parents=True,exist_ok=True)
    folders=sorted(p for p in source.iterdir() if p.is_dir() and not p.name.startswith('.'))
    # v16: Ignore only truly empty team placeholder directories.
    folders=[p for p in folders if any(p.iterdir())]
    if any(not TEAM.fullmatch(p.name) for p in folders):
        raise ValueError('src/team에는 team_01 ~ team_12 폴더만 두십시오.')
    # Complete validation before any write: a failed submission cannot partially update the registry.
    specs=[]
    for folder in folders:
        team_id=folder.name
        meta=metadata(folder,team_id)
        digest=files_digest(folder)
        registered=target/team_id
        if registered.exists() and not (registered/MARKER).exists():
            raise ValueError(f'{registered}: 기존 수동 접수 파일을 덮어쓰지 않습니다. 중복 접수 상태를 확인하십시오.')
        previous=None
        if (registered/MARKER).exists():
            previous=json.loads((registered/MARKER).read_text(encoding='utf-8'))
            if previous.get('source')!=f'src/team/{team_id}':
                raise ValueError(f'{registered}: v7 관리 마커가 일치하지 않습니다.')
        specs.append((folder,registered,meta,digest,previous))
    result=[]
    for folder,registered,meta,digest,previous in specs:
        team_id=folder.name
        if previous and previous.get('sha256')==digest:
            status='UNCHANGED'
        elif dry_run:
            status='WOULD_UPDATE' if previous else 'WOULD_IMPORT'
        else:
            # Existing normalize() from competition intake is used to preserve its contract.
            from submission import normalize
            with tempfile.TemporaryDirectory(prefix='urrc_v7_',dir=str(target)) as temporary:
                staged=Path(temporary)/'stage'
                identifier=normalize(folder,staged)
                if identifier!=team_id:raise ValueError(f'접수된 ID가 다릅니다: {identifier}')
                if not (staged/'race_entry.json').exists():raise ValueError('접수 메타데이터가 없습니다.')
                marker={'source':f'src/team/{team_id}','sha256':digest,'version':7}
                (staged/MARKER).write_text(json.dumps(marker,ensure_ascii=False)+'\n',encoding='utf-8')
                if registered.exists():
                    # Replace only entries previously imported by v7. No backup is written.
                    old=Path(temporary)/'previous'
                    registered.rename(old)
                    try:staged.rename(registered)
                    except BaseException:
                        old.rename(registered)
                        raise
                else:
                    staged.rename(registered)
            status='UPDATED' if previous else 'IMPORTED'
        result.append((team_id,status))
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description='URRC v7 src/team -> race/entrants')
    p.add_argument('command',nargs='?',choices=('sync','check'),default='sync')
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2])
    args=p.parse_args(argv)
    root=args.root.expanduser().resolve()
    if not (root/'race/scripts/submission.py').is_file():
        p.error('기존 URRC F1 저장소가 아닙니다.')
    sys.path.insert(0,str(root/'race/scripts'))
    try:
        rows=sync(root,dry_run=args.command=='check')
        print(f'[v7] src/team 참가자 {len(rows)}개')
        for team_id,status in rows:print(f'  {team_id}: {status}')
        if not rows:print('[v7] 참가자 제출 전입니다. src/team/team_01 ~ team_12 등 제출 폴더를 넣으십시오.')
        return 0
    except (OSError,ValueError,KeyError,json.JSONDecodeError) as exc:
        print('[v7] 접수 오류:',exc,file=sys.stderr)
        return 2

if __name__=='__main__':raise SystemExit(main())
