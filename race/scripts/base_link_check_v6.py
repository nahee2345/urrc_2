#!/usr/bin/env python3
"""Conservative base_link geometric center check; no false automatic PASS for articulated meshes."""
import argparse,json,math,xml.etree.ElementTree as ET
from pathlib import Path

def xyz(text):
    values=[float(x) for x in (text or '').split()]
    return values[:3] if len(values)>=3 else [0.,0.,0.]

def extents(geometry):
    if geometry is None:return None
    box=geometry.find('box')
    if box is not None:
        s=xyz(box.findtext('size'))
        return s if all(x>0 for x in s) else None
    sphere=geometry.find('sphere')
    if sphere is not None:
        r=float(sphere.findtext('radius'));return [2*r]*3
    cylinder=geometry.find('cylinder')
    if cylinder is not None:
        r=float(cylinder.findtext('radius'));h=float(cylinder.findtext('length'));return [2*r,2*r,h]
    return None

def check(path,base='base_link',xy_tolerance=.015,z_tolerance=.025):
    doc=ET.parse(path).getroot();warnings=[];errors=[];boxes=[]
    if doc.tag=='robot':
        links=doc.findall('link'); names={x.get('name') for x in links}
        if base not in names:errors.append(f'{base} 링크 없음')
        # URDF fixed and movable joints complicate the full-vehicle coordinate transforms.
        warnings.append('URDF 조인트 좌표 변환 미적용: 전체 중심은 실제 Gazebo 모델에서 재검수 필요')
        for link in links:
            if link.get('name')!=base:continue
            for item in link.findall('collision'):
                ex=extents(item.find('geometry'))
                if ex is None:warnings.append('base_link collision 메시 형상: 수동 검사');continue
                origin=item.find('origin');pos=xyz(origin.get('xyz') if origin is not None else None)
                if origin is not None and any(abs(float(v))>1e-8 for v in (origin.get('rpy') or '0 0 0').split()):warnings.append('회전된 collision: 수동 검사')
                boxes.append((pos,ex))
    else:
        model=doc if doc.tag=='model' else doc.find('model')
        if model is None:raise ValueError('SDF model 없음')
        links=model.findall('link'); names={x.get('name') for x in links}
        if base not in names:errors.append(f'{base} 링크 없음')
        base_link=next((x for x in links if x.get('name')==base),None)
        base_pose=xyz(base_link.findtext('pose')) if base_link is not None else [0.,0.,0.]
        if len(links)>1:warnings.append('복수 링크: 관절/상대좌표 변환에 대한 수동 검사 필요')
        for link in links:
            link_pose=xyz(link.findtext('pose'))
            if len((link.findtext('pose') or '').split())>=6:warnings.append('회전 링크: 수동 검사')
            for item in link.findall('collision'):
                ex=extents(item.find('geometry'))
                if ex is None:warnings.append('메시 collision: 전체 중심 수동 검사 필요');continue
                pos=xyz(item.findtext('pose'))
                if len((item.findtext('pose') or '').split())>=6:warnings.append('회전 collision: 수동 검사')
                boxes.append(([link_pose[i]+pos[i]-base_pose[i] for i in range(3)],ex))
    if not boxes:warnings.append('분석 가능한 collision 형상이 없어 중심 자동 판정 불가')
    center=None
    if boxes:
        lo=[min(p[i]-d[i]/2 for p,d in boxes) for i in range(3)]
        hi=[max(p[i]+d[i]/2 for p,d in boxes) for i in range(3)]
        center=[round((lo[i]+hi[i])/2,5) for i in range(3)]
        if abs(center[0])>xy_tolerance or abs(center[1])>xy_tolerance or abs(center[2])>z_tolerance:
            errors.append(f'base_link 원점과 분석 collision 외접상자 중심이 다름: {center}m')
    return {'status':'FAIL' if errors else 'REVIEW_REQUIRED' if warnings else 'GEOMETRIC_PASS', 'base_link':base,'collision_center_relative_to_base_m':center,'errors':errors,'warnings':sorted(set(warnings))}

def scan(root):
    out=[]
    for mf in sorted((root/'race/entrants').glob('*/race_entry.json')):
        try:
            data=json.loads(mf.read_text()); model=(mf.parent/data['vehicle_model_file']).resolve()
            if mf.parent.resolve() not in model.parents:raise ValueError('모델 경로가 참가 폴더 밖입니다')
            record=check(model,data.get('base_link_name','base_link'))
        except (OSError,ValueError,KeyError,ET.ParseError) as e:
            record={'status':'FAIL','errors':[str(e)]}
        out.append({'team':mf.parent.name,**record})
    return {'entrants':len(out),'teams':out,'note':'collision 중심 추정은 메시/조인트/센서 돌출/비대칭 물리 구조의 실제 중심 인증을 대체하지 않습니다.'}

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2]);p.add_argument('--report',type=Path);a=p.parse_args()
    result=scan(a.root.resolve());s=json.dumps(result,ensure_ascii=False,indent=2)
    print(s)
    if a.report:a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(s+'\n')
    return 2 if any(x['status']=='FAIL' for x in result['teams']) else 0
if __name__=='__main__':raise SystemExit(main())
