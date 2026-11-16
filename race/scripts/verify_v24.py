#!/usr/bin/env python3
"""Static v24 ZIP/folder validation. No ROS or Gazebo process required."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ALL=('monza','hungaroring','monaco','spa','cota','baku')
PRACTICE=('monza','hungaroring','monaco')
REQUIRED_COMMON=(
 'race_v24.sh', 'race_v23.sh', 'race_v22.sh', 'race_v21.sh',
 'race/scripts/v24_dispatch.py', 'race/scripts/compatibility_v23.py',
 'race/scripts/race_manager.py','race/scripts/race_camera_ready_v21.py',
 'race/scripts/stage_preview_v17.py','race/scripts/vehicle_preflight_v16.py',
 'src/urrc_track_gazebo/urrc_tracks/v10_assets.py',
 'src/urrc_track_gazebo/package.xml',
)

def check(root:Path)->list[str]:
 root=root.resolve();errors=[]
 profile=root/'race/v24_profile.json'
 if not profile.is_file():return ['v24_profile.json 없음']
 try:p=json.loads(profile.read_text(encoding='utf-8'))
 except (ValueError,OSError) as e:return ['프로필 오류: '+str(e)]
 mode=p.get('mode'); allowed=PRACTICE if mode=='practice' else ALL if mode=='official' else ()
 if not allowed:errors.append('배포 모드 오류')
 if set(p.get('tracks',[]))!=set(allowed):errors.append('배포 맵 목록 오류')
 for rel in REQUIRED_COMMON:
  if not (root/rel).is_file():errors.append('운영 파일 누락: '+rel)
 pkg=root/'src/urrc_track_gazebo'
 for name in ALL:
  exists=name in allowed
  paths=[pkg/'worlds'/f'{name}{suffix}' for suffix in ('.sdf','_race.sdf','_grid.gui.config','_overview.gui.config','_signal_active.sdf')]
  paths += [pkg/'models'/f'urrc_{name}'/'model.sdf',pkg/'tracks/processed'/f'{name}.json',pkg/'tracks/raw'/f'{name}.geojson',pkg/'tracks/config'/f'{name}.yaml',pkg/'launch'/f'{name}.launch.py']
  if exists:
   for q in paths:
    if not q.is_file():errors.append(f'{name}: 파일 누락: {q.relative_to(root)}')
   try:
    world=ET.parse(pkg/'worlds'/f'{name}_race.sdf').getroot()
    gantry=world.find("./world/model[@name='race_start_gantry']")
    if gantry is None or any(gantry.find(f"link/visual[@name='red_lens_{i}']") is None for i in range(1,6)):
     errors.append(f'{name}: 5등 신호등 누락')
    if any(s.get('type','').lower().find('camera')>=0 for s in world.findall('.//sensor')):
     errors.append(f'{name}: 맵 외부 카메라 남음')
    active=ET.parse(pkg/'worlds'/f'{name}_signal_active.sdf').getroot()
    if len(active.findall('./model/link/visual'))!=5:
     errors.append(f'{name}: 신호등 단일 소등 모델 오류')
    model=ET.parse(pkg/'models'/f'urrc_{name}'/'model.sdf').getroot()
    for c in ('road_collision','wall_outer_collision','wall_inner_collision'):
     node=model.find(f".//collision[@name='{c}']")
     if node is None:errors.append(f'{name}: 충돌체 없음: {c}')
    for uri in model.findall('.//mesh/uri'):
     txt=(uri.text or '').strip()
     prefix=f'model://urrc_{name}/'
     if txt.startswith(prefix):
      target=pkg/'models'/f'urrc_{name}'/txt[len(prefix):]
      if not target.is_file():errors.append(f'{name}: 메시 없음: {txt}')
   except (ET.ParseError,OSError) as e:errors.append(f'{name}: SDF/모델 파싱 오류: {e}')
  else:
   if (pkg/'models'/f'urrc_{name}').exists():errors.append(name+' 비공개 메시 잔존')
   for q in paths:
    if q.exists():errors.append('연습용 비공개 맵 누출: '+str(q.relative_to(root)))
   for d in ('raw','processed','config'):
    if any((pkg/'tracks'/d).glob(name+'*')):errors.append(name+' 비공개 트랙 데이터 잔존')
   if any((pkg/'worlds').glob(name+'*')):errors.append(name+' 비공개 월드 데이터 잔존')
 for i in range(1,13):
  t=root/'src/team'/f'team_{i:02d}'
  if not t.is_dir():errors.append('빈 팀 폴더 누락: '+t.name)
  elif any(x.name != '.gitkeep' for x in t.iterdir()):errors.append('배포본에 참가팀 코드 포함: '+t.name)
 for pth in (root/'race/entrants',root/'race/inbox'):
  if pth.exists() and any(x.name!='.gitkeep' for x in pth.iterdir()):
   errors.append('배포본에 기존 접수물/경기 기록 포함: '+str(pth.relative_to(root)))
 for rootdir in (root/'src/team',root/'race/scripts',pkg/'worlds',pkg/'models'):
  if rootdir.exists() and any(x.is_symlink() for x in rootdir.rglob('*')):
   errors.append('심볼릭 링크가 포함되어 있습니다: '+str(rootdir))
 return errors

def main():
 a=argparse.ArgumentParser();a.add_argument('workspace',nargs='?',default='.')
 args=a.parse_args();e=check(Path(args.workspace))
 if e:
  for x in e:print('[v24 FAIL]',x)
  return 2
 print('[v24 STATIC PASS]',Path(args.workspace).resolve())
 print('실제 ROS2/Gazebo 센서 수신·주행 테스트는 별도로 필요합니다.')
 return 0
if __name__=='__main__':sys.exit(main())
