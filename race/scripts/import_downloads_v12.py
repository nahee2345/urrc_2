#!/usr/bin/env python3
"""Prepare only team_01 and team_02 from ~/Downloads, without modifying originals.

Directory wins over same-name ZIP. Preflight both before replacing anything.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import shutil
import stat
import sys
import tempfile
import zipfile
import xml.etree.ElementTree as ET

SKIP={'build','install','log','.git','__pycache__','.pytest_cache','__MACOSX'}
TEAMS=('team_01','team_02')


def ignore(directory, names):
    return {n for n in names if n in SKIP or n.startswith('.DS_Store') or n=='Thumbs.db'}


def safe_unzip(source:Path, destination:Path):
    with zipfile.ZipFile(source) as z:
        infos=z.infolist()
        if len(infos)>25000 or sum(i.file_size for i in infos)>1_000_000_000:
            raise ValueError(f'{source}: ZIP 크기/파일 수 제한 초과')
        for i in infos:
            p=Path(i.filename)
            if p.is_absolute() or '..' in p.parts or not p.parts or '\\' in i.filename:
                raise ValueError(f'{source}: 안전하지 않은 ZIP 경로 {i.filename!r}')
            mode=i.external_attr>>16
            if stat.S_ISLNK(mode):
                raise ValueError(f'{source}: ZIP 심볼릭 링크 금지 {i.filename!r}')
            if any(x in SKIP for x in p.parts) or p.name.startswith('.DS_Store'):
                continue
            target=destination.joinpath(*p.parts)
            if i.is_dir():
                target.mkdir(parents=True,exist_ok=True)
            else:
                target.parent.mkdir(parents=True,exist_ok=True)
                with z.open(i) as src, target.open('wb') as out:
                    shutil.copyfileobj(src,out)


def manifest(folder):
    first=folder/'race_entry.json'
    if first.is_file():return first
    candidates=[]
    if (folder/'src').is_dir():candidates.extend((folder/'src').glob('*/race_entry.json'))
    if not candidates:candidates.extend(folder.glob('*/race_entry.json'))
    return candidates[0] if len(candidates)==1 else None


def unwrap(folder):
    current=folder
    for _ in range(5):
        # Unwrap a ZIP's single outer team_XX folder even when a nested manifest
        # would otherwise make it look like a valid submission root.
        if (current/'race_entry.json').is_file():return current
        if (current/'src').is_dir() and list((current/'src').glob('*/race_entry.json')):
            return current
        children=[p for p in current.iterdir() if p.name not in SKIP and not p.name.startswith('.')]
        dirs=[p for p in children if p.is_dir()]
        other=[p for p in children if not p.is_dir()]
        if len(dirs)==1 and not other:
            current=dirs[0]
            continue
        if manifest(current) is not None:return current
        raise ValueError(f'{current}: race_entry.json을 찾을 수 없거나 제출 폴더가 중복되었습니다.')
    raise ValueError(f'{folder}: 제출 폴더 중첩이 너무 깊습니다.')


def source_for(downloads,team):
    directory=downloads/team
    archive=downloads/(team+'.zip')
    if directory.is_dir() and not directory.is_symlink():
        return directory,'folder'
    if directory.exists():raise ValueError(f'{directory}: 폴더 형식이 아닙니다.')
    if archive.is_file() and not archive.is_symlink():
        return archive,'zip'
    raise FileNotFoundError(f'{team}: {directory} 또는 {archive} 파일을 찾지 못했습니다.')


def resolve_model_path(source_root, normalized_root, meta_file, data, team):
    path=normalized_root/data.get('vehicle_model_file','')
    if path.is_file() and path.resolve().is_relative_to(normalized_root.resolve()):return data
    old_name=Path(data.get('vehicle_model_file','model.sdf')).name
    models=sorted(normalized_root.joinpath('src').rglob(old_name))
    if len(models)!=1:
        raise ValueError(f'{team}: 메타데이터 vehicle_model_file 경로 오류이며, 모델 파일 {old_name!r} 후보가 {len(models)}개입니다.')
    data['vehicle_model_file']=models[0].relative_to(normalized_root).as_posix()
    meta_file.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(f'[v12 IMPORT] {team}: 차량 모델 경로 자동 정규화 -> {data["vehicle_model_file"]}')
    return data


def verify_ros_package(normalized, data, team):
    names=[]
    for p in (normalized/'src').rglob('package.xml'):
        try:
            names.append(ET.parse(p).getroot().findtext('name','').strip())
        except ET.ParseError as exc:
            raise ValueError(f'{team}: package.xml 오류 {p}: {exc}')
    if data.get('ros_package') not in names:
        raise ValueError(f'{team}: ros_package={data.get("ros_package")!r}와 제출된 ROS 패키지 {names}가 일치하지 않습니다.')


def prepare(root,downloads,work,team):
    raw,kind=source_for(downloads,team)
    material=work/team/team; material.mkdir(parents=True)
    if kind=='zip':safe_unzip(raw,material)
    else:
        for item in raw.rglob('*'):
            if item.is_symlink():
                raise ValueError(f'{team}: 제출 폴더에 심볼릭 링크가 있습니다: {item}')
        shutil.copytree(raw,material,dirs_exist_ok=True,ignore=ignore,symlinks=False)
    src=unwrap(material)
    m=manifest(src)
    if m is None:raise ValueError(f'{team}: race_entry.json 없음')
    data=json.loads(m.read_text(encoding='utf-8'))
    if not isinstance(data,dict):raise ValueError(f'{team}: race_entry.json 객체가 아닙니다.')
    before=data.get('entry_id')
    data['entry_id']=team
    m.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    # Use the organizer's actual submission normalization and model-path check.
    sys.path.insert(0,str(root/'race/scripts'))
    from submission import normalize
    normalized=work/team/'normalized'
    try:
        returned=normalize(src,normalized)
    except ValueError as exc:
        if 'vehicle_model_file' not in str(exc):raise
        data=resolve_model_path(src,normalized,m,data,team)
        shutil.rmtree(normalized)
        returned=normalize(src,normalized)
    if returned!=team:raise ValueError(f'{team}: 접수 ID 변환 실패')
    verify_ros_package(normalized,data,team)
    if not (normalized/data['vehicle_model_file']).is_file():
        raise ValueError(f'{team}: 차량 모델 파일 없음')
    print(f'[v12 IMPORT] {team}: {kind}={raw.name} / entry_id {before!r}->{team} / '
          f'package={data["ros_package"]} / model={data["vehicle_model_file"]}',flush=True)
    return src


def perform(root:Path,downloads:Path,import_now:bool):
    root=root.expanduser().resolve()
    downloads=downloads.expanduser().resolve()
    destination=root/'src/team'
    if not (root/'race/scripts/submission.py').is_file() or not (root/'race_v12.sh').is_file():
        raise ValueError('v12가 설치된 대회장 워크스페이스가 아닙니다.')
    if not downloads.is_dir():raise ValueError(f'Downloads 폴더 없음: {downloads}')
    # Work in the destination volume, but do not write to source directories.
    destination.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.urrc_v12_import_',dir=destination) as d:
        work=Path(d)
        sources={t:prepare(root,downloads,work,t) for t in TEAMS}
        if not import_now:
            print('[v12 IMPORT] 사전 검증 PASS. 실제 복사는 --import 옵션으로 실행하십시오.')
            return
        placed=[]
        try:
            for team in TEAMS:
                target=destination/team
                previous=work/(team+'_previous')
                if target.exists():target.rename(previous)
                try:sources[team].rename(target)
                except BaseException:
                    if previous.exists():previous.rename(target)
                    raise
                placed.append((target,previous))
        except BaseException:
            for target,previous in reversed(placed):
                if target.is_dir():shutil.rmtree(target)
                if previous.exists():previous.rename(target)
            raise
    print('[v12 IMPORT] 적용 완료: team_01 / team_02. team_11 및 기타 운영 파일은 그대로입니다.')


def main():
    p=argparse.ArgumentParser(description='URRC v12: ~/Downloads 팀 1,2만 안전 등록')
    p.add_argument('--root',type=Path,default=Path.home()/'urrc_f1_ws')
    p.add_argument('--downloads',type=Path,default=Path.home()/'Downloads')
    action=p.add_mutually_exclusive_group()
    action.add_argument('--check',action='store_true',help='기본 동작: 사전 검증만')
    action.add_argument('--import',dest='do_import',action='store_true',help='검증 후 실제 팀 폴더 반영')
    a=p.parse_args()
    try:perform(a.root,a.downloads,a.do_import)
    except (OSError,ValueError,KeyError,zipfile.BadZipFile,json.JSONDecodeError,ET.ParseError) as e:
        print('[v12 IMPORT ERROR]',e,file=sys.stderr);return 2
    return 0

if __name__=='__main__':raise SystemExit(main())
