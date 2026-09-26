#!/usr/bin/env python3
"""Test lens emission in a running practice world, without participant cars."""
import argparse
import os
import time
from pathlib import Path
from race_manager import lights_all, light_request

p=argparse.ArgumentParser()
p.add_argument('state',choices=('on','off','sequence'))
p.add_argument('--track',default='monza')
p.add_argument('--hold',type=float,default=3)
a=p.parse_args()
os.environ.setdefault('GZ_PARTITION',f'urrc_practice_{os.getuid()}')
root=Path(__file__).resolve().parents[2]
if a.state=='on':lights_all(a.track,root,True)
elif a.state=='off':lights_all(a.track,root,False)
else:
    try:
        lights_all(a.track,root,False)
        for i in range(5):
            light_request(a.track,i,True,root);time.sleep(1)
        time.sleep(max(0,a.hold))
    finally:lights_all(a.track,root,False)
