#!/usr/bin/env python3
"""Verify a fresh clone or GitHub ZIP without ROS, Gazebo, or third-party Python modules."""
import csv
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parent


def main():
    config=json.loads((ROOT/'release.json').read_text())
    package=ROOT/'src/urrc_track_gazebo'
    tracks=config['tracks']
    actual={p.stem for p in (package/'worlds').glob('*.sdf') if not p.stem.endswith('_race')}
    if actual!=set(tracks):raise ValueError(f'배포 맵 목록 불일치: {actual}')
    count=0
    for line in (ROOT/'SHA256SUMS').read_text().splitlines():
        digest,name=line.split('  ',1)
        f=ROOT/name
        if not f.is_file() or hashlib.sha256(f.read_bytes()).hexdigest()!=digest:
            raise ValueError(f'파일 누락/변경: {name}')
        count+=1
    for track in tracks:
        meta=json.loads((package/f'tracks/processed/{track}.json').read_text())
        if len(meta['grid'])<12:raise ValueError(f'{track}: 12대 그리드 부족')
        for slot in meta['grid']:
            if not all(math.isfinite(float(slot[k])) for k in ('x','y','road_z','yaw')):
                raise ValueError(f'{track}: 유효하지 않은 스폰 좌표')
        with (package/f'tracks/processed/{track}_centerline.csv').open() as f:
            for row in csv.DictReader(f):
                if not all(math.isfinite(float(v)) for v in row.values()):
                    raise ValueError(f'{track}: 유효하지 않은 중심선 좌표')
        world=ET.parse(package/f'worlds/{track}_race.sdf').getroot()
        if world.find('world').get('name')!=track:raise ValueError(f'{track}: 월드 이름 불일치')
        for i in range(1,6):
            if world.find(f".//model[@name='race_start_gantry']/link[@name='gantry_visuals']/visual[@name='red_lens_{i}']") is None:
                raise ValueError(f'{track}: LED 렌즈 누락 {i}')
    for f in package.rglob('*.sdf'):
        root=ET.parse(f).getroot()
        for uri in root.findall('.//uri'):
            value=uri.text or ''
            if value.startswith('model://'):
                target=package/'models'/value[len('model://'):]
            elif '://' not in value:
                target=f.parent/value
            else:
                raise ValueError(f'외부 URI: {f}: {value}')
            if not target.exists():raise ValueError(f'리소스 누락: {target}')
    for f in ROOT.rglob('*'):
        if any(part in {'.git','build','install','log','build_standalone','install_standalone','__pycache__','results','entrants','inbox'} for part in f.relative_to(ROOT).parts):continue
        if f.suffix in {'.py','.sh','.sdf','.config'} and f.is_file():
            if re.search(r'/(?:home|workspace)/',f.read_text(errors='replace')):
                raise ValueError(f'PC 절대 경로 발견: {f.relative_to(ROOT)}')
    print(f'PASS: {config["role"]}, {len(tracks)}개 맵, {count}개 파일 체크섬, 리소스·스폰·LED·경로 검사')
    print('ROS/Gazebo 실제 렌더링·차량 주행은 실행 PC에서 별도 확인해야 합니다.')

if __name__=='__main__':
    try:main()
    except Exception as exc:print(f'FAIL: {exc}',file=sys.stderr);sys.exit(1)
