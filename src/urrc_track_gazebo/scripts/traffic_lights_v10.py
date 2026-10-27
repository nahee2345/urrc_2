#!/usr/bin/env python3
"""Auto-start five URRC v10.8-style reds after the *existing* Monza world is ready.

Only for the participant's bare Gazebo world. Race and practice modes call
the SAME atomic_signal.sequence directly and do not spawn this entrypoint.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "race" / "scripts"))
from atomic_signal import schedule, sequence


def wait_ready(track, timeout=70):
    wanted={f"/world/{track}/visual_config",f"/world/{track}/create",f"/world/{track}/remove"}
    deadline=time.monotonic()+timeout
    seen=set()
    while time.monotonic()<deadline:
        try:
            result=subprocess.run(["gz","service","-l"],capture_output=True,
                                  text=True,timeout=5,env=os.environ.copy())
            if result.returncode==0:
                seen=set(result.stdout.splitlines())
                if wanted<=seen:
                    print("[READY] Gazebo signal services available",flush=True)
                    return
        except (OSError,subprocess.SubprocessError) as exc:
            print(f"[WAIT] Gazebo services not available yet: {exc}",flush=True)
        time.sleep(.35)
    raise TimeoutError(f"Missing Gazebo services: {sorted(wanted-seen)}")


def main(argv=None):
    parser=argparse.ArgumentParser()
    parser.add_argument("--track",choices=("monza",),required=True)
    parser.add_argument("--seed",type=int)
    parser.add_argument("--dry-run",action="store_true")
    args=parser.parse_args(argv)
    if args.dry_run:
        gaps,hold=schedule(args.seed)
        print(f"[PLAN] 5 red LEDs, gaps={gaps}, hold={hold:.3f}, total={sum(gaps)+hold:.3f}")
        return 0
    try:
        wait_ready(args.track)
        sequence(args.track,ROOT,env=os.environ.copy(),seed=args.seed)
        return 0
    except (RuntimeError,TimeoutError,OSError,subprocess.SubprocessError) as exc:
        print(f"[URRC SIGNAL ERROR] {exc}",file=sys.stderr,flush=True)
        return 2


if __name__=="__main__":
    sys.exit(main())
