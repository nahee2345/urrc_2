#!/usr/bin/env python3
"""Five randomized sequential red lights and one-model all-off (URRC v10.8)."""
import argparse
import json
import os
from pathlib import Path
import random
import re
import shutil
import subprocess
import sys
import time

TRACKS = ("monza", "spa", "monaco", "cota", "hungaroring", "baku")
_ACTIVE = "urrc_start_signal_active"


def schedule(seed=None, total=None):
    rng=random.Random(seed)
    total=rng.uniform(3.35,5.05) if total is None else float(total)
    if not 3.0 < total < 6.0: raise ValueError("First red to all-off must be strictly 3..6 seconds")
    hold=rng.uniform(0.40,min(0.90,total-2.0))
    budget=total-hold
    weights=[rng.uniform(0.45,2.5) for _ in range(4)]
    gaps=[.35+(budget-1.40)*w/sum(weights) for w in weights]
    first=rng.uniform(.15,.28)
    on=[first]
    for gap in gaps:on.append(on[-1]+gap)
    return {"on_s":on,"off_s":on[-1]+hold,"target_first_on_to_off_s":total,"hold_s":hold}


def _call(track,service,reqtype,payload,optional=False):
    r=subprocess.run(["gz","service","-s",f"/world/{track}/{service}",
        "--reqtype",reqtype,"--reptype","gz.msgs.Boolean",
        "--timeout","5000","--req",payload],
        capture_output=True,text=True,timeout=8)
    ok=r.returncode==0 and bool(re.search(r"data:\s*true",r.stdout))
    if not ok and not optional:
        raise RuntimeError(f"{service} failed: {r.stdout[-250:]} {r.stderr[-250:]}")
    return ok


def wait_ready(track,timeout=70,check_cameras=True,ready_file=None):
    """Do not activate signals before the Gazebo service and both cameras exist."""
    end=time.monotonic()+timeout
    service=f"/world/{track}/visual_config"
    wanted={f"/urrc/{track}/spectator/{name}/image" for name in ("start","overview")}
    last="waiting"
    while time.monotonic()<end:
        r=subprocess.run(["gz","service","-l"],capture_output=True,text=True,timeout=6)
        service_ready=r.returncode==0 and service in r.stdout.splitlines()
        topics=set()
        if service_ready and check_cameras:
            try:
                t=subprocess.run(["gz","topic","-l"],capture_output=True,text=True,timeout=6)
                topics=set(t.stdout.splitlines()) if t.returncode==0 else set()
            except subprocess.SubprocessError:
                pass
        cameras_ok=not check_cameras or wanted<=topics
        vehicle_ok=ready_file is None or Path(ready_file).is_file()
        if service_ready and cameras_ok and vehicle_ok:
            print("[READY] Gazebo visual service and start/overview camera topics ready",flush=True)
            return
        last=f"service={service_ready} missing_cameras={sorted(wanted-topics) if check_cameras else []} vehicles={vehicle_ok}"
        time.sleep(.5)
    raise TimeoutError("Cannot start red lights; prerequisites missing: "+last)


def _create(track):
    # Generated relative to this installed package; no fixed machine path.
    template=Path(__file__).resolve().parents[1]/"worlds"/f"{track}_signal_active.sdf"
    if not template.is_file():raise FileNotFoundError(template)
    _call(track,"remove","gz.msgs.Entity",f'name: "{_ACTIVE}" type: 2',optional=True)
    time.sleep(.30)
    _call(track,"create","gz.msgs.EntityFactory",
          "sdf_filename: "+json.dumps(str(template)))
    time.sleep(.30)


def _red(track,index):
    request=(f'name: "active_red_lens_{index}" parent_name: "signal_active_visuals" '
        'material { ambient { r: 0.12 g: 0.005 b: 0.006 a: 1 } '
        'diffuse { r: 0.12 g: 0.006 b: 0.008 a: 1 } '
        'specular { r: 0.02 g: 0.02 b: 0.02 a: 1 } '
        'emissive { r: 1 g: 0.008 b: 0.008 a: 1 } }')
    _call(track,"visual_config","gz.msgs.Visual",request)
    return time.monotonic()


def _off(track):
    # Entire temporary five-lens model disappears in ONE entity operation.
    _call(track,"remove","gz.msgs.Entity",f'name: "{_ACTIVE}" type: 2')
    return time.monotonic()


def run(track,seed=None,total=None,dry_run=False,check_cameras=True,ready_file=None,timeout=70):
    if track not in TRACKS:raise ValueError(track)
    plan=schedule(seed,total)
    if dry_run:
        print(json.dumps({"track":track,"plan":plan},indent=2))
        return plan
    if not shutil.which("gz"):raise RuntimeError("Gazebo gz not found")
    wait_ready(track,timeout,check_cameras,ready_file)
    first=None
    _create(track)
    started=time.monotonic()
    try:
        for index,at in enumerate(plan["on_s"],1):
            time.sleep(max(0,started+at-time.monotonic()))
            stamp=_red(track,index)
            if first is None:first=stamp
            print(f"RED {index}/5 at {stamp-started:.3f}s",flush=True)
        end=max(started+plan["off_s"],first+plan["target_first_on_to_off_s"])
        time.sleep(max(0,end-time.monotonic()))
        elapsed=_off(track)-first
        print(f"LIGHTS OUT first_on_to_off={elapsed:.3f}s model_remove=single_request",flush=True)
        if not 3.0 < elapsed < 6.0:
            print("[WARN] Gazebo service latency exceeded 3..6s target",flush=True)
        return elapsed
    except BaseException:
        _call(track,"remove","gz.msgs.Entity",f'name: "{_ACTIVE}" type: 2',optional=True)
        raise


def main(argv=None):
    p=argparse.ArgumentParser()
    p.add_argument("--track",required=True,choices=TRACKS)
    p.add_argument("--seed",type=int)
    p.add_argument("--total",type=float)
    p.add_argument("--dry-run",action="store_true")
    p.add_argument("--ready-file",type=Path)
    p.add_argument("--timeout",type=float,default=70)
    p.add_argument("--skip-camera-check",action="store_true",help="Only for diagnosis")
    a=p.parse_args(argv)
    try:
        run(a.track,a.seed,a.total,a.dry_run,not a.skip_camera_check,a.ready_file,a.timeout)
        return 0
    except (RuntimeError,ValueError,TimeoutError,subprocess.SubprocessError,OSError) as exc:
        print("[URRC SIGNAL ERROR]",str(exc),file=sys.stderr)
        return 2

if __name__=="__main__":
    sys.exit(main())
