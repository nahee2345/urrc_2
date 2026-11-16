#!/usr/bin/env python3
"""URRC v4 offline entrant inspection + online spectator diagnostics.
Does not modify entrants, Gazebo worlds, or competition timing.
"""
import argparse
import json
import math
import re
import shutil
import subprocess
import sys
import time
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

MAX_DIMS=(0.55,0.25,0.18)
CAMERAS=('start','overview','sector2','sector3')
TRACKS=('monza','spa','monaco','cota','hungaroring','baku')
REQUIRED=('entry_id','ros_package','algorithm_launch_file','vehicle_model_file','speed_topic','steering_topic','lidar_topic')

def safe_path(folder, rel):
    p=(folder/rel).resolve()
    if folder.resolve() not in p.parents: raise ValueError('제출 폴더 밖의 경로')
    return p

def geometry_extent(geom):
    """Geometric primitive local AABB only. Meshes need manual checks."""
    if geom.find('box') is not None:
        a=geom.findtext('box/size','').split()
        return [float(i) for i in a] if len(a)==3 else None
    if geom.find('sphere') is not None:
        r=float(geom.findtext('sphere/radius'))
        return [2*r]*3
    if geom.find('cylinder') is not None:
        r=float(geom.findtext('cylinder/radius')); length=float(geom.findtext('cylinder/length'))
        return [2*r,2*r,length]
    return None

def pose_xyz(s):
    if not s:return (0.,0.,0.)
    a=[float(x) for x in s.split()]
    return tuple(a[:3]) if len(a)>=3 else (0.,0.,0.)

def inspect_model(path, fmt, base):
    root=ET.parse(path).getroot(); warnings=[]; errors=[]
    # URDF model geometry/origin and SDF pose differ; both can include rotations.
    if fmt=='urdf' or root.tag=='robot':
        links=root.findall('link'); seen={x.get('name') for x in links}
        shapes=[]
        for link in links:
            for item in link.findall('collision')+link.findall('visual'):
                g=item.find('geometry')
                if g is None:continue
                extent=geometry_extent(g)
                if extent is None: warnings.append('메시/지원되지 않는 형상은 수동 외형 검수 필요');continue
                origin=item.find('origin'); p=pose_xyz(origin.get('xyz') if origin is not None else None)
                if origin is not None and origin.get('rpy','0 0 0')!='0 0 0':warnings.append('회전한 형상이 있어 외형 수동 검사 필요')
                shapes.append((p,extent))
        warnings.append('URDF 링크 간 관절 변환을 반영한 전체 외형 검사는 미지원')
    else:
        model=root if root.tag=='model' else root.find('./model')
        if model is None: raise ValueError('SDF 최상위 model이 없습니다')
        links=model.findall('link');seen={x.get('name') for x in links};shapes=[]
        for link in links:
            link_pose=pose_xyz(link.findtext('pose'))
            for item in link.findall('collision')+link.findall('visual'):
                g=item.find('geometry')
                if g is None:continue
                extent=geometry_extent(g)
                if extent is None:warnings.append('메시/지원되지 않는 형상은 수동 외형 검수 필요');continue
                p=pose_xyz(item.findtext('pose'));shapes.append((tuple(a+b for a,b in zip(link_pose,p)),extent))
                if len((item.findtext('pose') or '').split())>=6 or len((link.findtext('pose') or '').split())>=6:
                    warnings.append('회전한 형상이 있어 외형 수동 검사 필요')
        if model.findtext('static','false').strip().lower() in ('true','1'):errors.append('static 차량 모델')
    if base not in seen:errors.append(f'차체 기준 링크 없음: {base}')
    if not shapes:warnings.append('자동 측정 가능한 기하 형상이 없습니다')
    # Dimensions of a single primitive are a safe lower-bound check when no rotation.
    for _,ext in shapes:
        if any(n>d+1e-6 for n,d in zip(ext,MAX_DIMS)):
            errors.append('단일 형상 치수가 규격 초과: '+str([round(x,4) for x in ext]))
    warnings.append('자동 PASS는 전체 부피/센서 위치 적합성을 보증하지 않습니다')
    return sorted(set(errors)),sorted(set(warnings))

def inspect_entries(root):
    folder=root/'race/entrants'; report=[]; seen_ids=set();packages=set();topics=set()
    for team in sorted(folder.iterdir()) if folder.is_dir() else []:
        mf=team/'race_entry.json'
        if not team.is_dir() or not mf.is_file():continue
        errors=[];warnings=[]; entry={}
        try:
            entry=json.loads(mf.read_text(encoding='utf-8'))
            for key in REQUIRED:
                if not entry.get(key):errors.append(f'필수 항목 누락: {key}')
            eid=entry.get('entry_id','')
            if eid!=team.name:errors.append('entry_id와 폴더 이름 불일치')
            if not re.fullmatch(r'team_(?:0[1-9]|10)',eid):errors.append('team_01~team_10 이름 규격 불일치')
            if eid in seen_ids:errors.append('중복 팀 ID')
            seen_ids.add(eid)
            pkg=entry.get('ros_package')
            if pkg in packages:errors.append('중복 ROS 패키지 이름')
            packages.add(pkg)
            for k in ('speed_topic','steering_topic','lidar_topic'):
                v=entry.get(k)
                if v in topics:errors.append('다른 팀과 중복된 토픽: '+str(v))
                topics.add(v)
            model_rel=entry.get('vehicle_model_file')
            if model_rel:
                path=safe_path(team,model_rel)
                if not path.is_file():errors.append('차량 모델 파일 없음')
                else:
                    e,w=inspect_model(path,entry.get('vehicle_model_format','sdf'),entry.get('base_link_name','base_link'))
                    errors+=e;warnings+=w
            launch=entry.get('algorithm_launch_file','')
            if launch and pkg:
                candidates=list((team/'src').rglob('launch/'+launch)) if (team/'src').exists() else []
                if not candidates:errors.append('알고리즘 launch 파일 없음')
            if not list((team/'src').rglob('package.xml')) if (team/'src').exists() else True:
                errors.append('ROS 패키지 package.xml 없음')
            warnings.append('실제 LiDAR·카메라 수신, 조향, 바퀴 접촉, 빌드·주행은 별도 검증 필요')
        except (OSError,ValueError,ET.ParseError,TypeError,KeyError) as exc:errors.append(f'검사 예외: {exc}')
        report.append({'team':team.name,'status':'FAIL' if errors else 'REVIEW','errors':sorted(set(errors)),'warnings':sorted(set(warnings))})
    return {'kind':'vehicle_inspection','entrant_count':len(report),'status':'NO_ENTRANTS' if not report else ('FAIL' if any(x['errors'] for x in report) else 'REVIEW_REQUIRED'),'teams':report,'note':'정적 검사입니다. 물리 시뮬레이션과 빌드 검증을 대체하지 않습니다.'}

def run_cmd(args, timeout=8):
    try:
        p=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
        return p.returncode,(p.stdout+'\n'+p.stderr).strip()
    except (OSError,subprocess.TimeoutExpired) as exc:return 124,str(exc)

def image_received(topic):
    try:
        with tempfile.TemporaryFile() as out:
            proc=subprocess.run(['gz','topic','-e','-t',topic,'-n','1'],stdout=out,stderr=subprocess.DEVNULL,timeout=9)
            return proc.returncode==0 and out.tell()>0
    except (OSError,subprocess.TimeoutExpired):
        return False

def camera_topics(track):
    return {k:f'/urrc/{track}/spectator/{k}/image' for k in CAMERAS}

def inspect_cameras(track, sample=False):
    if not shutil.which('gz'):
        return {'status':'BLOCKED','reason':'Gazebo CLI gz 미설치','track':track}
    code,output=run_cmd(['gz','topic','-l'])
    if code!=0:return {'status':'BLOCKED','reason':output,'track':track}
    observed=set(output.splitlines()); details={}
    for key,topic in camera_topics(track).items():
        present=topic in observed
        state='TOPIC_PRESENT' if present else 'MISSING'
        if sample and present:
            # Check an actual image message, never capture full image contents in report.
            state='FRAME_RECEIVED' if image_received(topic) else 'NO_FRAME'
        details[key]={'topic':topic,'state':state}
    ok=all(v['state'] in ('TOPIC_PRESENT','FRAME_RECEIVED') for v in details.values())
    return {'kind':'camera_inspection','track':track,'status':'PASS' if ok else 'FAIL','cameras':details,
            'vehicle_follow_1_to_0':'NOT_IMPLEMENTED','note':'U/I/O/P만 실제 구현. 1~0은 기존 관전 코드에서 예약 상태.'}

def main(argv=None):
    p=argparse.ArgumentParser(description='URRC v4 entrant/camera inspection')
    p.add_argument('command',choices=('vehicles','cameras'))
    p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2]);p.add_argument('--track',choices=TRACKS)
    p.add_argument('--sample',action='store_true',help='각 카메라에서 영상 메시지 1개 수신 검사')
    p.add_argument('--report',type=Path)
    a=p.parse_args(argv);root=a.root.resolve()
    if a.command=='cameras' and not a.track:p.error('cameras는 --track이 필요합니다')
    data=inspect_entries(root) if a.command=='vehicles' else inspect_cameras(a.track,a.sample)
    s=json.dumps(data,ensure_ascii=False,indent=2)
    if a.report:
        a.report.parent.mkdir(parents=True,exist_ok=True)
        a.report.write_text(s+'\n',encoding='utf-8')
    print(s)
    return 2 if data['status'] in ('FAIL','BLOCKED') else 0
if __name__=='__main__':sys.exit(main())
