"""Adapter to the EXACT v10.8 traffic_lights_v10.py of urrc_f1_ws.

The source script, visual commands, randomized schedule, and one-model
lights-out are shared. Race, practice, and manual tests route through it.
No legacy 5-on / 5-staggered-off commands remain here.
"""
import importlib.util
import os
from pathlib import Path
import subprocess
import sys

ACTIVE="urrc_start_signal_active"


def _source(root):
    script=Path(root)/"src"/"urrc_track_gazebo"/"scripts"/"traffic_lights_v10.py"
    if not script.is_file():
        raise FileNotFoundError(script)
    return script


def _module(root):
    spec=importlib.util.spec_from_file_location("urrc_f1_v108_lights",_source(root))
    if spec is None or spec.loader is None:
        raise RuntimeError("Cannot load v10.8 F1 signal module")
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def schedule(seed=None):
    plan=_module(Path(__file__).resolve().parents[2]).schedule(seed=seed)
    on=plan["on_s"]
    return [on[i+1]-on[i] for i in range(4)],plan["hold_s"]


def all_off(track,root,*,env=None,missing_ok=True):
    script=_module(root)
    # Same one-entity removal request as official urrc_f1_ws v10.8.
    if env:
        # Manual control in the current process is normally already on the
        # right Gazebo partition. Competition and practice sequences use a
        # separate process with explicit env below.
        current=os.environ.get("GZ_PARTITION")
        if env.get("GZ_PARTITION") != current:
            raise RuntimeError("Manual signal GZ_PARTITION mismatch")
    return script._call(track,"remove","gz.msgs.Entity",
                        f'name: "{ACTIVE}" type: 2',optional=missing_ok)


def lamp(track,index,enabled,root,*,env=None):
    if not enabled or not 0<=index<5:
        raise ValueError("Use lamp index 0..4 for ON, all_off for OFF")
    script=_module(root)
    if env and env.get("GZ_PARTITION")!=os.environ.get("GZ_PARTITION"):
        raise RuntimeError("Manual signal GZ_PARTITION mismatch")
    if index==0:
        script._create(track)
    return script._red(track,index+1)


def lights_all(track,root,on,*,env=None):
    if not on:
        return all_off(track,root,env=env)
    # Used only for manual 'on' and diagnostics; race/practice both use
    # the exact canonical run() via sequence(), not this helper.
    for i in range(5):
        lamp(track,i,True,root,env=env)


def sequence(track,root,*,env=None,seed=None):
    """Launch exact urrc_f1_ws v10.8 controller with partition preserved.

    urrc_2 has no spectator camera sensors. We bypass ONLY that readiness
    condition; the ON/OFF code and random schedule are byte-identical.
    """
    command=[sys.executable,str(_source(root)),"--track",track,
             "--skip-camera-check"]
    if seed is not None:
        command.extend(["--seed",str(seed)])
    result=subprocess.run(command,env=env,check=False)
    if result.returncode:
        raise RuntimeError(f"v10.8 signal sequence failed (exit={result.returncode})")
    return result.returncode
