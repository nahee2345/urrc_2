#!/usr/bin/env python3
"""Actual Gazebo checks, explicit NOT_RUN when the required binary is missing."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time

TRACKS = ("monza",)
VARIANTS = ("practice", "race")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--package", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path, default=Path.cwd() / "smoke_results")
    parser.add_argument("--timeout", type=float, default=60)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    if not shutil.which("gz"):
        result = {"status": "NOT_RUN", "reason": "Gazebo gz command not installed", "tracks": list(TRACKS), "variants": list(VARIANTS)}
        (args.output / "runtime_report.json").write_text(json.dumps(result, indent=2) + "\n")
        print("NOT_RUN: Gazebo gz command not installed"); return 2
    version = subprocess.run(["gz", "sim", "--versions"], text=True, capture_output=True, timeout=15)
    for name in TRACKS:
      for variant in VARIANTS:
        world_name = name if variant == "practice" else f"{name}_race"
        env = os.environ.copy()
        env["GZ_SIM_RESOURCE_PATH"] = str(args.package / "models") + os.pathsep + env.get("GZ_SIM_RESOURCE_PATH", "")
        env["GZ_PARTITION"] = f"urrc_smoke_{os.getpid()}_{world_name}"
        cmd = ["gz", "sim", "-s", "-r", "-v", "4", "--iterations", "1000", str(args.package / "worlds" / f"{world_name}.sdf")]
        logfile = args.output / f"{world_name}.log"
        with logfile.open("w") as log:
            process = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            timeout = False
            try:
                code = process.wait(timeout=args.timeout)
            except subprocess.TimeoutExpired:
                timeout = True
                os.killpg(process.pid, signal.SIGTERM)
                try: code = process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); code = process.wait()
        text = logfile.read_text(errors="replace")
        errors = [line for line in text.splitlines() if re.search(r"\[Err\]|Unable to find|Error Code|Failed to load", line, re.I)]
        # The explicit world-loaded message prevents an empty successful process
        # from being reported as a load PASS.
        loaded = bool(re.search(r"Loaded (world|level)|Serving world|World.*initialized", text, re.I))
        status = "PASS" if code == 0 and not timeout and not errors and loaded else "FAIL"
        results.append({"track": name, "variant": variant, "status": status, "exit_code": code, "timeout": timeout,
                        "world_load_message": loaded, "errors": errors, "log": str(logfile)})
        print(f"{world_name}: {status}", flush=True)
    (args.output / "runtime_report.json").write_text(json.dumps({"gazebo_version": version.stdout.strip(), "results": results,
        "scope": "Server load and 1000 iterations. GUI, GPU LiDAR and moving-vehicle collision tests are separate."}, indent=2) + "\n")
    return 0 if all(r["status"] == "PASS" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
