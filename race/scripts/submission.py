"""Portable participant folder normalisation; Python standard library only."""
import json
import re
import shutil
from pathlib import Path

SKIP={"build","install","log",".git","__pycache__",".pytest_cache"}


def normalize(source, destination):
    source=Path(source).resolve(); destination=Path(destination)
    # ZIPs may wrap the submission in one team directory.
    while not (source/'race_entry.json').is_file() and not (source/'package.xml').is_file():
        children=[p for p in source.iterdir() if p.name not in SKIP and not p.name.startswith('.')]
        if len(children)==1 and children[0].is_dir() and children[0].name!='src':
            source=children[0]
        else:
            break
    manifests=[source/'race_entry.json'] if (source/'race_entry.json').is_file() else list(source.glob('*/race_entry.json'))
    if not manifests and (source/'src').is_dir():
        manifests=list((source/'src').glob('*/race_entry.json'))
    if len(manifests)!=1:
        raise ValueError('제출 루트 또는 대표 패키지에 race_entry.json 하나가 필요합니다.')
    data=json.loads(manifests[0].read_text())
    eid=data.get('entry_id','')
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]{0,31}',eid):
        raise ValueError('entry_id 형식 오류')
    for p in source.rglob('*'):
        if p.is_symlink():
            raise ValueError(f'제출물에 심볼릭 링크를 사용할 수 없습니다: {p}')
    destination.mkdir(parents=True,exist_ok=True)
    ignore=shutil.ignore_patterns(*SKIP)
    if (source/'package.xml').is_file():
        shutil.copytree(source,destination/'src'/source.name,ignore=ignore)
    elif (source/'src').is_dir():
        shutil.copytree(source/'src',destination/'src',ignore=ignore)
    else:
        shutil.copytree(source,destination/'src',ignore=ignore)
    (destination/'race_entry.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    model=(destination/data.get('vehicle_model_file','')).resolve()
    if destination.resolve() not in model.parents or not model.is_file():
        raise ValueError('vehicle_model_file은 정규화된 팀 폴더 기준 src/<패키지>/... 경로여야 합니다.')
    if not list((destination/'src').rglob('package.xml')):
        raise ValueError('src에 ROS 패키지 package.xml이 없습니다.')
    return eid
