#!/usr/bin/env python3
"""Static checks, optional full six-circuit generation without Gazebo."""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from urrc_tracks.v10_assets import TRACKS, enhance_all


def check(package, source_only=False):
    errors=[]
    for name in TRACKS:
        model=package/"models"/f"urrc_{name}"/"model.sdf"
        base=package/"worlds"/f"{name}.sdf"
        race=package/"worlds"/f"{name}_race.sdf"
        if not all(p.is_file() for p in (model,base,race)):
            errors.append(f"{name}: missing base resources")
            continue
        if source_only:
            gate=ET.parse(race).find("world/model[@name='race_start_gantry']")
            if gate is None or len(gate.findall(".//visual[@name='red_lens_1']"))!=1:
                errors.append(f"{name}: original five-lens gantry missing")
            continue
        active=package/"worlds"/f"{name}_signal_active.sdf"
        if not active.is_file():
            errors.append(f"{name}: missing v10.8 active signal")
            continue
        lenses=ET.parse(active).findall(".//visual")
        if {lens.get("name") for lens in lenses}!={f"active_red_lens_{i}" for i in range(1,6)}:
            errors.append(f"{name}: active model needs exactly five bulbs")
        for suffix in ("","_race"):
            world=ET.parse(package/"worlds"/f"{name}{suffix}.sdf")
            cameras=world.findall(".//model[@name='urrc_v10_spectator']/link/sensor")
            if len(cameras)!=4:
                errors.append(f"{name}{suffix}: need U/I/O/P cameras")
    for error in errors:print("[FAIL]",error)
    return not errors


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--package",type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument("--source-only",action="store_true")
    parser.add_argument("--prepare-all",action="store_true")
    args=parser.parse_args()
    if args.prepare_all:
        print("Generating all six local UV meshes and v10.8 worlds...")
        enhance_all(args.package)
    sys.exit(0 if check(args.package,args.source_only) else 1)
