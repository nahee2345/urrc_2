#!/usr/bin/env python3
"""Launch exact-grid pace cars with original five-light sequence and spectator cameras."""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
if str(HERE) not in sys.path:sys.path.insert(0,str(HERE))
from routes import TRACKS, generate


def build_plugin(output):
    if not shutil.which('cmake'):
        raise RuntimeError('CMake not installed: sudo apt install cmake g++ libgz-sim8-dev')
    source=HERE/'plugin'
    output.mkdir(parents=True,exist_ok=True)
    subprocess.run(['cmake','-S',str(source),'-B',str(output),'-DCMAKE_BUILD_TYPE=Release'],check=True)
    subprocess.run(['cmake','--build',str(output),'--parallel','2'],check=True)
    path=output/'liburrc_pace_cars.so'
    if not path.is_file():raise RuntimeError('Gazebo pace plugin build failed: '+str(path))
    return path


def terminate(proc):
    if proc is None or proc.poll() is not None:return
    try:os.killpg(proc.pid,signal.SIGTERM)
    except ProcessLookupError:return
    try:proc.wait(timeout=7)
    except subprocess.TimeoutExpired:
        try:os.killpg(proc.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        proc.wait()


def start_signal(track,env,generated):
    signal_script=generated/'resources/urrc_track_gazebo/scripts/traffic_lights_v10.py'
    return subprocess.Popen([sys.executable,str(signal_script),'--track',track],
                            cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                            text=True,bufsize=1,start_new_session=True)


def fire_start(track,env):
    """Send a reliable start burst only after successful five-light model removal."""
    for attempt in range(3):
        cmd=['gz','topic','-t',f'/urrc/{track}/pace/start',
             '-m','gz.msgs.Boolean','-p','data: true']
        result=subprocess.run(cmd,cwd=ROOT,env=env,text=True,
                              capture_output=True,timeout=12)
        if result.returncode:
            raise RuntimeError('Gazebo pace start event publish failed: '+result.stderr[-300:])
        if attempt<2:time.sleep(0.1)


def confirmed_all_off(line: str) -> bool:
    """Only the original signal script's successful all-off service result authorizes departure."""
    return line.startswith("LIGHTS OUT ") and "model_remove=single_request" in line


def main(argv=None):
    parser=argparse.ArgumentParser(description='URRC Monza / Hungaroring fixed-lane pace cars')
    parser.add_argument('track',choices=TRACKS)
    parser.add_argument('--prepare-only',action='store_true',help='Generate and inspect world; do not build or run simulator')
    parser.add_argument('--server',action='store_true',help='Gazebo headless server, no GUI')
    args=parser.parse_args(argv)
    output=ROOT/'generated/pace_cars'/args.track
    build=ROOT/'generated/pace_cars/build'
    plugin=build/'liburrc_pace_cars.so'
    if not args.prepare_only:
        if not shutil.which('gz'):parser.error('Gazebo Harmonic is missing: gz command not found')
        plugin=build_plugin(build)
    print(f'[PACE] Preparing independent {args.track} world and 2 cars ...',flush=True)
    config=generate(ROOT,args.track,output,plugin)
    for car in config['cars']:
        print(f"[PACE] Grid {car['grid']}: {car['speed_mps']:.0f}m/s, exact start XYZ={car['start_xyz']}, lane offset={car['signed_lane_offset_m']:.3f}m",flush=True)
    if args.prepare_only:
        print('[PACE] World generated only, no Gazebo runtime test.',flush=True)
        return 0
    env=os.environ.copy()
    env['GZ_PARTITION']=f'urrc_pace_{os.getuid()}'
    env['GZ_SIM_RESOURCE_PATH']=config['resource_path']+os.pathsep+env.get('GZ_SIM_RESOURCE_PATH','')
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(plugin.parent)+os.pathsep+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    world=config['world_file']
    print(f'[PACE] Track={args.track}, partition={env["GZ_PARTITION"]}',flush=True)
    print('[PACE] Waiting for all 5 red lights to turn OFF before starting cars.',flush=True)
    simcmd=['gz','sim','-r','-v','3']
    if args.server:simcmd+=['-s','--headless-rendering']
    else:simcmd+=['--gui-config',str(output/'resources/urrc_track_gazebo/worlds'/f'{args.track}_grid.gui.config')]
    simcmd.append(world)
    log=output/'gazebo.log'
    sim=lights=None
    try:
        with log.open('wb') as loghandle:
            sim=subprocess.Popen(simcmd,cwd=ROOT,env=env,stdout=loghandle,stderr=subprocess.STDOUT,start_new_session=True)
            # A published readiness topic proves the native plugin loaded and advertised.
            ready=f'/urrc/{args.track}/pace/start/ready'
            deadline=time.monotonic()+50
            while time.monotonic()<deadline:
                if sim.poll() is not None:
                    raise RuntimeError(f'Gazebo exited before pace plugin readiness; inspect {log}')
                try:
                    check=subprocess.run(['gz','topic','-l'],env=env,stdout=subprocess.PIPE,
                                         stderr=subprocess.DEVNULL,text=True,timeout=6)
                    if check.returncode==0 and ready in check.stdout.splitlines():
                        break
                except subprocess.TimeoutExpired:
                    pass
                time.sleep(0.5)
            else:
                raise RuntimeError(f'Pace plugin not ready ({ready}); inspect {log}')
            print('[PACE] Native pace controller ready.',flush=True)
            lights=start_signal(args.track,env,output)
            fired=False
            assert lights.stdout is not None
            for line in lights.stdout:
                print('[SIGNAL] '+line.rstrip(),flush=True)
                if confirmed_all_off(line) and not fired:
                    fire_start(args.track,env)
                    fired=True
                    print('[PACE] All 5 lights OFF. Grid 02 = 10 m/s, Grid 09 = 20 m/s.',flush=True)
            status=lights.wait()
            if status!=0:raise RuntimeError(f'Original traffic lights failed, exit {status}. Gazebo log: {log}')
            if not fired:raise RuntimeError('Five-light all-OFF confirmation missing; cars remain stationary')
            print('[PACE] Signals complete. Cars continue lapping until Ctrl+C.',flush=True)
            result=sim.wait()
            return result
    except KeyboardInterrupt:
        print('\n[PACE] Stopping ...',flush=True)
        return 130
    finally:
        terminate(lights)
        terminate(sim)

if __name__=='__main__':
    try:sys.exit(main())
    except (RuntimeError,subprocess.CalledProcessError,subprocess.TimeoutExpired,OSError) as exc:
        print('[PACE ERROR] '+str(exc),file=sys.stderr)
        sys.exit(2)
