"""Portable v10.8 circuit visuals: five-light gantry, spectator cameras, asphalt.

Idempotent, map-derived, local generation. Does not touch collision meshes or
the car/race algorithms. Generated large OBJ assets stay on the local computer.
"""
from __future__ import annotations
from copy import deepcopy
from pathlib import Path
import csv
import json
import math
import random
import re
import shutil
import struct
import xml.etree.ElementTree as ET
import zlib

TRACKS = ("monza", "hungaroring")
SPECTATOR_MODES = ("start", "overview", "sector2", "sector3")
CAMERA_HFOV = {"start":1.20, "overview":1.30, "sector2":1.04, "sector3":1.04}
VERSION = "urrc-v10.8-spectator-zoom-2026-10"


def _write_xml(root, filename):
    ET.indent(root, "  ")
    ET.ElementTree(root).write(filename, encoding="utf-8", xml_declaration=True)


def _set(parent, tag, value):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = str(value)


def _material(visual, image, roughness):
    material = visual.find("material")
    if material is None: material = ET.SubElement(visual, "material")
    for tag, value in (("ambient","1 1 1 1"),("diffuse","1 1 1 1"),
                       ("specular","0.01 0.01 0.01 1")):
        _set(material, tag, value)
    pbr = material.find("pbr")
    if pbr is None: pbr = ET.SubElement(material, "pbr")
    metal = pbr.find("metal")
    if metal is None: metal = ET.SubElement(pbr, "metal")
    for tag,value in (("albedo_map",image),("metalness","0"),("roughness",roughness)):
        _set(metal, tag, value)


def _png(filename, base, seed):
    """Deterministic mild asphalt grain, entirely standard-library PNG."""
    if filename.is_file(): return
    filename.parent.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    width = height = 128
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        for x in range(width):
            grain = rng.randrange(-9, 10)
            for ch in base:
                raw.append(max(0,min(255,ch+grain)))
    def chunk(tag, data):
        return struct.pack(">I",len(data))+tag+data+struct.pack(">I",zlib.crc32(tag+data)&0xffffffff)
    ihdr=struct.pack(">IIBBBBB",width,height,8,2,0,0,0)
    filename.write_bytes(b"\x89PNG\r\n\x1a\n"+chunk(b"IHDR",ihdr)
                         +chunk(b"IDAT",zlib.compress(bytes(raw),9))+chunk(b"IEND",b""))


def _textures(package):
    assets=package/"assets"
    _png(assets/"asphalt.png",(42,44,47),1807)
    _png(assets/"offroad.png",(73,79,68),1808)


def _road_uv(source, target, tile=0.55):
    """OBJ visual UVs, no change to original road collision OBJ."""
    if target.is_file() and target.stat().st_mtime >= source.stat().st_mtime: return
    temporary=target.with_suffix(".obj.tmp")
    with source.open(encoding="utf-8") as src, temporary.open("w",encoding="utf-8") as dst:
        for line in src:
            if line.startswith("mtllib "):
                dst.write("mtllib road_asphalt.mtl\n")
            elif line.startswith("usemtl "):
                dst.write("usemtl URRCAsphalt\n")
            elif line.startswith("v "):
                dst.write(line)
                xyz=line.split()
                dst.write(f"vt {float(xyz[1])/tile:.7f} {float(xyz[2])/tile:.7f}\n")
            elif line.startswith("f "):
                faces=[]
                for corner in line.split()[1:]:
                    bits=corner.split("/")
                    index=int(bits[0]); normal=bits[-1] if len(bits)==3 and bits[-1] else bits[0]
                    faces.append(f"{index}/{index}/{normal}")
                dst.write("f "+" ".join(faces)+"\n")
            else:
                dst.write(line)
    temporary.replace(target)


def _ground_obj(path):
    if path.is_file(): return
    extent=1000
    uv=2000/.55
    path.write_text(
        "mtllib road_asphalt.mtl\no asphalt_ground\nusemtl URRCAsphalt\n"
        "v -1000 -1000 0\nv 1000 -1000 0\nv 1000 1000 0\nv -1000 1000 0\n"
        f"vt 0 0\nvt {uv:.6f} 0\nvt {uv:.6f} {uv:.6f}\nvt 0 {uv:.6f}\n"
        "vn 0 0 1\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n",encoding="ascii")


def _course_samples(package,name,meta):
    source=package/"tracks"/"processed"/f"{name}_centerline.csv"
    if source.is_file():
        with source.open(newline="", encoding="utf-8") as f:
            pts=[(float(r["x_m"]),float(r["y_m"]),float(r["z_m"])) for r in csv.DictReader(f)]
        if len(pts)>=9:return pts
    return [(float(r["x"]),float(r["y"]),float(r["road_z"])) for r in meta["grid"]]


def _camera_fit(points,hfov,margin,pad):
    """Fit rotated XY bounds inside actual 16:9 camera horizontal FOV."""
    cx=(min(p[0] for p in points)+max(p[0] for p in points))/2
    cy=(min(p[1] for p in points)+max(p[1] for p in points))/2
    hx=math.tan(hfov/2)
    hy=hx/(16/9)
    best=None
    for deg in range(0,180,2):
        angle=math.radians(deg)
        cs,sn=math.cos(angle),math.sin(angle)
        us=[(x-cx)*cs+(y-cy)*sn for x,y,_ in points]
        vs=[-(x-cx)*sn+(y-cy)*cs for x,y,_ in points]
        distance=max((max(us)-min(us)+2*margin)/(2*hx),
                     (max(vs)-min(vs)+2*margin)/(2*hy))
        if best is None or distance<best[0]:
            u=(max(us)+min(us))/2
            v=(max(vs)+min(vs))/2
            best=(distance,cx+u*cs-v*sn,cy+u*sn+v*cs,angle)
    dist,tx,ty,angle=best
    return tx,ty,max(z for _,_,z in points)+dist*pad,math.pi/2,angle+math.pi/2


def _window(points,fraction,half=10.0):
    distance=[0.]
    for a,b in zip(points,points[1:]):
        distance.append(distance[-1]+math.hypot(a[0]-b[0],a[1]-b[1]))
    center=distance[-1]*fraction
    indices=[i for i,d in enumerate(distance) if abs(d-center)<=half]
    if len(indices)<3:
        mid=min(range(len(points)),key=lambda i:abs(distance[i]-center))
        indices=list(range(max(0,mid-2),min(len(points),mid+3)))
    return [points[i] for i in indices]


def _camera_pose(meta,points,mode):
    if mode=="start":
        first=meta.get("start_finish",meta["grid"][0])
        x0,y0=float(first["x"]),float(first["y"])
        z0=float(first.get("z",first.get("road_z",0)))
        yaw=float(first["yaw"]);fx,fy=math.cos(yaw),math.sin(yaw)
        x=x0+fx*1.7+fy*4.8;y=y0+fy*1.7-fx*4.8;z=z0+3.8
        tx,ty,tz=x0-1.4*fx,y0-1.4*fy,z0+0.45
        return x,y,z,math.atan2(z-tz,max(0.01,math.hypot(tx-x,ty-y))),math.atan2(ty-y,tx-x)
    if mode=="overview":return _camera_fit(points,CAMERA_HFOV[mode],3.0,1.08)
    if mode=="sector2":return _camera_fit(_window(points,.36),CAMERA_HFOV[mode],2.2,1.12)
    if mode=="sector3":return _camera_fit(_window(points,.70),CAMERA_HFOV[mode],2.2,1.12)
    raise ValueError(mode)


def _add_cameras(world,name,meta,points):
    for previous in list(world.findall("model")):
        if previous.get("name")=="urrc_v10_spectator":
            world.remove(previous)
    model=ET.SubElement(world,"model",name="urrc_v10_spectator")
    ET.SubElement(model,"static").text="true"
    for mode in SPECTATOR_MODES:
        x,y,z,pitch,yaw=_camera_pose(meta,points,mode)
        link=ET.SubElement(model,"link",name=f"{mode}_camera_mount")
        ET.SubElement(link,"pose").text=f"{x:.6f} {y:.6f} {z:.6f} 0 {pitch:.6f} {yaw:.6f}"
        sensor=ET.SubElement(link,"sensor",name=f"{mode}_camera",type="camera")
        ET.SubElement(sensor,"always_on").text="true"
        ET.SubElement(sensor,"update_rate").text="16" if mode=="start" else "6"
        ET.SubElement(sensor,"visualize").text="false"
        ET.SubElement(sensor,"topic").text=f"/urrc/{name}/spectator/{mode}/image"
        camera=ET.SubElement(sensor,"camera")
        ET.SubElement(camera,"horizontal_fov").text=f"{CAMERA_HFOV[mode]:.2f}"
        image=ET.SubElement(camera,"image")
        ET.SubElement(image,"width").text="800" if mode=="start" else "640"
        ET.SubElement(image,"height").text="450" if mode=="start" else "360"
        ET.SubElement(image,"format").text="R8G8B8"
        clip=ET.SubElement(camera,"clip")
        ET.SubElement(clip,"near").text="0.1"
        ET.SubElement(clip,"far").text="5000"


def _active_model(gate,target):
    """One temporary model owns five illuminated overlay visuals."""
    root=ET.Element("sdf",version="1.9")
    model=ET.SubElement(root,"model",name="urrc_start_signal_active")
    ET.SubElement(model,"static").text="true"
    ET.SubElement(model,"pose").text=gate.findtext("pose")
    link=ET.SubElement(model,"link",name="signal_active_visuals")
    for index in range(1,6):
        original=gate.find(f"link[@name='gantry_visuals']/visual[@name='red_lens_{index}']")
        if original is None or original.find("geometry/box/size") is None:
            raise RuntimeError(f"Expected five independent lenses (missing {index})")
        visual=deepcopy(original)
        visual.set("name",f"active_red_lens_{index}")
        pose=[float(v) for v in visual.findtext("pose").split()]
        pose[0]-=.020
        visual.find("pose").text=" ".join(f"{v:.7f}" for v in pose)
        visual.find("geometry/box/size").text=".008 .110 .120"
        mat=visual.find("material")
        if mat is None:mat=ET.SubElement(visual,"material")
        _set(mat,"ambient","0.015 0.005 0.006 1")
        _set(mat,"diffuse","0.015 0.005 0.006 1")
        _set(mat,"emissive","0 0 0 1")
        link.append(visual)
    _write_xml(root,target)


def enhance_track(package,name):
    package=Path(package)
    if name not in TRACKS:raise ValueError(name)
    _textures(package)
    mesh=package/"models"/f"urrc_{name}"/"meshes"
    model_path=mesh.parent/"model.sdf"
    _road_uv(mesh/"road.obj",mesh/"road_textured.obj")
    _ground_obj(mesh/"asphalt_ground.obj")
    for texture in ("asphalt.png","offroad.png"):
        shutil.copyfile(package/"assets"/texture,mesh/texture)
    (mesh/"road_asphalt.mtl").write_text(
        "newmtl URRCAsphalt\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\nmap_Kd asphalt.png\n",encoding="ascii")
    doc=ET.parse(model_path)
    road=doc.find(".//visual[@name='road_visual']")
    _set(road.find("geometry/mesh"),"uri",f"model://urrc_{name}/meshes/road_textured.obj")
    _material(road,f"model://urrc_{name}/meshes/asphalt.png","0.94")
    _write_xml(doc.getroot(),model_path)
    worlds=package/"worlds"
    race=ET.parse(worlds/f"{name}_race.sdf")
    raceworld=race.find("world")
    gate=raceworld.find("model[@name='race_start_gantry']")
    if gate is None:raise RuntimeError(f"{name}: missing starting gantry")
    for i in range(1,6):
        lens=gate.find(f"link[@name='gantry_visuals']/visual[@name='red_lens_{i}']")
        if lens is None:raise RuntimeError(f"{name}: missing red lens {i}")
        mat=lens.find("material")
        _set(mat,"ambient","0.015 0.005 0.006 1")
        _set(mat,"diffuse","0.015 0.005 0.006 1")
        _set(mat,"emissive","0 0 0 1")
    _active_model(gate,worlds/f"{name}_signal_active.sdf")
    for lamp in list(raceworld.findall("light")):
        if re.fullmatch(r"race_red_[1-5]",lamp.get("name","")):
            raceworld.remove(lamp)
    meta=json.loads((package/"tracks"/"processed"/f"{name}.json").read_text())
    points=_course_samples(package,name,meta)
    for suffix in ("","_race"):
        path=worlds/f"{name}{suffix}.sdf"
        tree=ET.parse(path)
        world=tree.find("world")
        if suffix=="":
            for old in list(world.findall("model")):
                if old.get("name")=="race_start_gantry":world.remove(old)
            world.append(deepcopy(gate))
        else:
            world= raceworld
            tree=race
        _add_cameras(world,name,meta,points)
        visual=world.find("model[@name='ground']/link/visual[@name='ground_visual']")
        geometry=visual.find("geometry")
        for child in list(geometry):geometry.remove(child)
        ET.SubElement(ET.SubElement(geometry,"mesh"),"uri").text=f"model://urrc_{name}/meshes/asphalt_ground.obj"
        _material(visual,f"model://urrc_{name}/meshes/offroad.png","1.0")
        _write_xml(tree.getroot(),path)
    return name


def enhance_all(package, tracks=TRACKS):
    return [enhance_track(package,name) for name in tracks]
