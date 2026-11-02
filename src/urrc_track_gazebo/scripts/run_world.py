#!/usr/bin/env python3
"""Run exactly one local world. Runtime dependencies: Python stdlib and gz."""
import argparse
import fcntl
import json
import os
import random
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile

TRACKS = ("monza", "hungaroring")


def choose_track(mode, seed=None):
    if mode == "random":return random.Random(seed).choice(TRACKS)
    if mode not in TRACKS:
        raise ValueError("Unknown track: " + mode)
    return mode


def command_for(package, name, *, server=False, view="overview", iterations=None, lights=True):
    cmd = ["gz", "sim", "-r", "-v", "3"]
    if server:
        cmd.append("-s")
    else:
        cmd.extend(["--gui-config", str(package / "worlds" / f"{name}_{view}.gui.config")])
    if iterations is not None:
        cmd.extend(["--iterations", str(iterations)])
    cmd.append(str(package / "worlds" / f"{name}{'_race' if lights else ''}.sdf"))
    return cmd


def run(args=None):
    parser = argparse.ArgumentParser(description="URRC Monza 1/5 circuit")
    parser.add_argument("track", choices=(*TRACKS,"random"))
    parser.add_argument("--seed",type=int)
    parser.add_argument("--no-lights", action="store_true", help="Open the world without the start gantry")
    parser.add_argument("--server", action="store_true", help="Server only, no GUI")
    parser.add_argument("--view", choices=("overview", "grid"), default="overview")
    parser.add_argument("--iterations", type=int, help="Stop after this many simulation iterations")
    parser.add_argument("--dry-run", action="store_true", help="Show selection and command without starting Gazebo")
    parser.add_argument("--package", type=Path, default=Path(__file__).resolve().parents[1], help=argparse.SUPPRESS)
    opts = parser.parse_args(args)
    if opts.iterations is not None and opts.iterations <= 0:
        parser.error("--iterations must be positive")
    package = opts.package.resolve()
    name = choose_track(opts.track,opts.seed)
    world = package / "worlds" / f"{name}{'_race' if not opts.no_lights else ''}.sdf"
    model = package / "models" / ("urrc_" + name) / "model.sdf"
    if not world.is_file() or not model.is_file():
        parser.exit(2, f"Missing installed resources: {world}\n")
    cmd = command_for(package, name, server=opts.server, view=opts.view, iterations=opts.iterations, lights=not opts.no_lights)
    env = os.environ.copy()
    env["GZ_SIM_RESOURCE_PATH"] = str(package / "models") + os.pathsep + env.get("GZ_SIM_RESOURCE_PATH", "")
    if opts.dry_run:
        print(json.dumps({"selected_track": name, "command": cmd,
                          "GZ_SIM_RESOURCE_PATH": env["GZ_SIM_RESOURCE_PATH"]}, indent=2))
        return 0
    if not shutil.which("gz"):
        parser.exit(2, "Gazebo 'gz' command not found. See README_KO.md for 24.04 / 26.04 installation.\n")
    try:
        version = subprocess.run(["gz", "sim", "--versions"], capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired:
        parser.exit(2, "Gazebo version check timed out.\n")
    if version.returncode:
        parser.exit(2, "'gz' exists but Gazebo Sim is unavailable. Check installed Gazebo packages.\n")
    print(f"[URRC] track={name}  scale=1/5  walls=0.30m  grid=20", flush=True)
    print(f"[URRC] Gazebo Sim: {version.stdout.strip() or version.stderr.strip()}", flush=True)
    # Per-user lock prevents our two entry points from opening concurrent worlds.
    # It never kills another application, Gazebo instance, or a previous session.
    lock_dir = Path(tempfile.gettempdir()) / f"urrc-f1-{os.getuid()}"
    lock_dir.mkdir(mode=0o700, exist_ok=True)
    with (lock_dir / "world.lock").open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(3, "Another URRC world is running. Close that window / use Ctrl+C first.\n")
        env.setdefault("GZ_PARTITION", f"urrc_practice_{os.getuid()}")
        log_dir = Path(env.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "urrc_f1"
        log_dir.mkdir(parents=True, exist_ok=True)
        (log_dir / "last_run.json").write_text(json.dumps({"track": name, "command": cmd}, indent=2) + "\n")

        # First use on a new computer: generate local UV meshes and four cameras.
        # No internet, fixed home path, or regenerated collision files required.
        active = package / "worlds" / f"{name}_signal_active.sdf"
        textured = package / "models" / f"urrc_{name}" / "meshes" / "road_textured.obj"
        if (not active.is_file() or not textured.is_file()
                or "urrc_v10_spectator" not in world.read_text(encoding="utf-8")):
            import sys as _sys
            _sys.path.insert(0, str(package))
            from urrc_tracks.v10_assets import enhance_track
            print("[URRC v10.8] Preparing map, five-light signal and spectator cameras",flush=True)
            enhance_track(package,name)

        process = subprocess.Popen(cmd, env=env, start_new_session=True)
        lights_process = None
        lights_log = None
        if not opts.no_lights and opts.iterations is None:
            lights_log = (log_dir / "lights_v10.log").open("w")
            lights_process = subprocess.Popen(
                [sys.executable,str(package/"scripts"/"traffic_lights_v10.py"),
                 "--track",name],env=env,stdout=lights_log,
                stderr=subprocess.STDOUT,start_new_session=True)
        def stop(signum, frame):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        old_int, old_term = signal.signal(signal.SIGINT, stop), signal.signal(signal.SIGTERM, stop)
        try:
            return process.wait()
        finally:
            if lights_process is not None and lights_process.poll() is None:
                try: os.killpg(lights_process.pid, signal.SIGTERM)
                except ProcessLookupError: pass
                try: lights_process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(lights_process.pid, signal.SIGKILL)
                    lights_process.wait()
            if lights_log is not None:
                lights_log.close()
            signal.signal(signal.SIGINT, old_int); signal.signal(signal.SIGTERM, old_term)
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()


if __name__ == "__main__":
    sys.exit(run())
