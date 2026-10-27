#!/usr/bin/env python3
"""Manual signal-only diagnostics. Does not start vehicles."""
import argparse
import os
from pathlib import Path
from atomic_signal import all_off, lights_all, sequence

def main():
    p = argparse.ArgumentParser()
    p.add_argument('state', choices=('on','off','sequence'))
    p.add_argument('--track', default='monza')
    p.add_argument('--hold', type=float, default=3, help='Reserved; sequence uses a randomized 3..6s envelope')
    a = p.parse_args()
    os.environ.setdefault('GZ_PARTITION', f'urrc_practice_{os.getuid()}')
    root = Path(__file__).resolve().parents[2]
    if a.state == 'on':
        lights_all(a.track, root, True)
    elif a.state == 'off':
        all_off(a.track, root)
    else:
        sequence(a.track, root)

if __name__ == '__main__':
    main()
