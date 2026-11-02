"""Produce two exact-grid, lane-offset, closed-loop pace car routes without moving source files."""
from __future__ import annotations
import bisect
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

TRACKS = ('monza', 'hungaroring')
GRID_SPEEDS = ((2, 10.0), (9, 20.0))


def centerline(path):
    with Path(path).open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 50:
        raise ValueError('Insufficient source centerline points')
    return [(float(r['x_m']), float(r['y_m']), float(r['z_m']),float(r['yaw_rad']),float(r['road_width_m'])) for r in rows]


def projection(points, grid):
    """Orthogonal XY projection; signed normal offset keeps original grid lane."""
    best = None
    px,py=grid['x'],grid['y']
    for i in range(len(points)):
        a=points[i]; b=points[(i+1)%len(points)]
        dx=b[0]-a[0];dy=b[1]-a[1]
        l2=dx*dx+dy*dy
        if l2<1e-12:continue
        t=max(0.0,min(1.0,((px-a[0])*dx+(py-a[1])*dy)/l2))
        x=a[0]+t*dx;y=a[1]+t*dy
        sq=(px-x)**2+(py-y)**2
        if best is None or sq<best[0]:
            length=math.sqrt(l2)
            # left is +90deg from track tangent
            normal=(-dy/length,dx/length)
            signed=(px-x)*normal[0]+(py-y)*normal[1]
            best=(sq,i,t,signed)
    if best is None:raise ValueError('Failed to project grid')
    dist,i,t,offset=best
    if dist>1.0:
        raise ValueError('Grid farther than 1m from centerline')
    if abs(offset)> min(points[i][4],points[(i+1)%len(points)][4])/2-0.14:
        raise ValueError('Vehicle lane offset is outside road edge')
    return i,t,offset


def offset_ring(points, offset):
    return [(x-offset*math.sin(yaw),y+offset*math.cos(yaw),z,yaw,w) for (x,y,z,yaw,w) in points]


def lane_route(points,grid):
    i,t,offset=projection(points,grid)
    ring=offset_ring(points,offset)
    # Exact original grid pose is the first waypoint, no recenter / teleport.
    path=[(grid['x'],grid['y'],grid['road_z'],grid['yaw'])]
    n=len(ring)
    for step in range(n):
        index=(i+1+step)%n
        p=ring[index]
        path.append(p[:4])
    path.append(path[0])
    # A lane-offset path may be slightly different from original centerline length.
    length=0.0
    points_out=[(0.0,*path[0])]
    for p1,p2 in zip(path,path[1:]):
        length+=math.dist(p1[:3],p2[:3])
        points_out.append((length,*p2))
    if length<100:raise ValueError('Suspiciously short route')
    return {'samples':points_out,'length_m':length,'offset_m':offset,'grid_index':i,'grid_t':t}


def add_car(world, grid, color):
    """A simple visible, sensor-detectable rigid car. No motor / AI model added."""
    slot=grid['position'];name=f'urrc_pace_grid_{slot:02d}'
    model=ET.SubElement(world, 'model',name=name)
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=(f"{grid['x']:.8f} {grid['y']:.8f} "
                                      f"{grid['road_z']+0.003:.8f} 0 0 {grid['yaw']:.8f}")
    link=ET.SubElement(model,'link',name='base_link')
    for part,p,shape,material in [
        ('body','0 0 0.106 0 0 0',('box','.50 .23 .135'),color),
        ('cabin','-0.045 0 0.180 0 0 0',('box','.24 .19 .045'),color),
        ('nose','0.205 0 0.09 0 0 0',('box','.10 .23 .035'),'0.03 0.03 0.035 1'),
    ]:
        visual=ET.SubElement(link,'visual',name=part)
        ET.SubElement(visual,'pose').text=p
        geom=ET.SubElement(visual,'geometry')
        ET.SubElement(ET.SubElement(geom,shape[0]),'size').text=shape[1]
        mat=ET.SubElement(visual,'material')
        ET.SubElement(mat,'ambient').text=material
        ET.SubElement(mat,'diffuse').text=material
    for x in (-.17,.17):
        for y in (-.125,.125):
            v=ET.SubElement(link,'visual',name=f"wheel_{'f' if x>0 else 'r'}_{'l' if y>0 else 'r'}")
            ET.SubElement(v,'pose').text=f'{x} {y} .04 1.57079632679 0 0'
            geom=ET.SubElement(v,'geometry')
            cyl=ET.SubElement(geom,'cylinder')
            ET.SubElement(cyl,'radius').text='.043'
            ET.SubElement(cyl,'length').text='.035'
            mat=ET.SubElement(v,'material')
            ET.SubElement(mat,'ambient').text='0.06 0.06 0.06 1'
            ET.SubElement(mat,'diffuse').text='0.06 0.06 0.06 1'
    # Kinematic moving collision geometry is for LiDAR and collision queries.
    coll=ET.SubElement(link,'collision',name='body_collision')
    ET.SubElement(coll,'pose').text='0 0 .11 0 0 0'
    geom=ET.SubElement(coll,'geometry')
    ET.SubElement(ET.SubElement(geom,'box'),'size').text='.50 .23 .16'
    return name


def generate(root, track, target, libfile):
    if track not in TRACKS:raise ValueError(track)
    pkg=Path(root)/'src/urrc_track_gazebo'
    meta=json.loads((pkg/'tracks/processed'/f'{track}.json').read_text(encoding='utf-8'))
    pts=centerline(pkg/'tracks/processed'/f'{track}_centerline.csv')
    out=Path(target)
    out.mkdir(parents=True,exist_ok=True)
    # Copy original map resources into an isolated generated folder.
    import shutil
    res=out/'resources'/'urrc_track_gazebo'
    (res/'worlds').mkdir(parents=True,exist_ok=True)
    (res/'models').mkdir(parents=True,exist_ok=True)
    (res/'tracks'/'processed').mkdir(parents=True,exist_ok=True)
    shutil.copytree(pkg/'models'/f'urrc_{track}',res/'models'/f'urrc_{track}',dirs_exist_ok=True)
    for suffix in ('.sdf','_race.sdf', '_grid.gui.config'):
        shutil.copy2(pkg/'worlds'/f'{track}{suffix}',res/'worlds'/f'{track}{suffix}')
    shutil.copy2(pkg/'tracks/processed'/f'{track}.json',res/'tracks/processed'/f'{track}.json')
    shutil.copy2(pkg/'tracks/processed'/f'{track}_centerline.csv',res/'tracks/processed'/f'{track}_centerline.csv')
    (res/'scripts').mkdir(exist_ok=True)
    shutil.copy2(pkg/'scripts/traffic_lights_v10.py',res/'scripts'/'traffic_lights_v10.py')
    import sys
    sys.path.insert(0,str(pkg))
    from urrc_tracks.v10_assets import enhance_track
    enhance_track(res,track)
    world_tree=ET.parse(res/'worlds'/f'{track}_race.sdf')
    world=world_tree.getroot().find('world')
    plugin=ET.SubElement(world,'plugin',filename=str(Path(libfile).resolve()),name='urrc::PaceCars')
    ET.SubElement(plugin,'start_topic').text=f'/urrc/{track}/pace/start'
    manifest={'track':track,'start_on':'all_five_lights_off','cars':[]}
    for slot,speed in GRID_SPEEDS:
        grid=meta['grid'][slot-1]
        route=lane_route(pts,grid)
        fname=out/f'grid_{slot:02d}.csv'
        with fname.open('w',encoding='utf-8',newline='') as f:
            w=csv.writer(f);w.writerow(('distance_m','x_m','y_m','road_z_m','yaw_rad'))
            for row in route['samples']:w.writerow([f'{v:.9f}' for v in row])
        name=add_car(world,grid, '0.85 0.13 0.11 1' if slot==2 else '0.12 0.35 0.95 1')
        c=ET.SubElement(plugin,'car')
        ET.SubElement(c,'name').text=name
        ET.SubElement(c,'speed_mps').text=str(speed)
        ET.SubElement(c,'route_file').text=str(fname.resolve())
        ET.SubElement(c,'z_offset_m').text='.003'
        manifest['cars'].append({'model':name,'grid':slot,'speed_mps':speed,
                                 'start_xyz':[grid['x'],grid['y'],grid['road_z']+.003],
                                 'start_yaw':grid['yaw'],'signed_lane_offset_m':route['offset_m'],
                                 'path_length_m':route['length_m'],
                                 'path_file':str(fname.resolve())})
    ET.indent(world_tree)
    outfile=res/'worlds'/f'{track}_pace.sdf'
    world_tree.write(outfile,encoding='utf-8',xml_declaration=True)
    manifest['world_file']=str(outfile.resolve())
    manifest['resource_path']=str((res/'models').resolve())
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return manifest
