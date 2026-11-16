"""Validate five independent traffic-signal lenses and visual services resources."""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
TRACKS=('monza','spa','monaco','cota','hungaroring','baku')

def check(root,track):
    pkg=Path(root)/'src/urrc_track_gazebo'
    path=pkg/'worlds'/f'{track}_race.sdf'
    failures=[]
    try:
        doc=ET.parse(path).getroot()
        gate=doc.find(".//model[@name='race_start_gantry']")
        if gate is None:
            failures.append('missing race_start_gantry');lens=[]
        else:
            lens=gate.findall("./link[@name='gantry_visuals']/visual")
        names={v.get('name') for v in lens}
        missing=[f'red_lens_{i}' for i in range(1,6) if f'red_lens_{i}' not in names]
        if missing:failures.append('missing lenses '+','.join(missing))
        lights=[v for v in doc.findall('.//world/light') if (v.get('name') or '').startswith('race_red_')]
        if len(lights) not in (0,5):failures.append(f'incorrect legacy lights count {len(lights)}')
        active=pkg/'worlds'/f'{track}_signal_active.sdf'
        if not lights and not active.is_file():failures.append('missing generated signal_active.sdf')
        if active.is_file():
            active_doc=ET.parse(active).getroot()
            found={x.get('name') for x in active_doc.findall('.//visual')}
            expected={f'active_red_lens_{i}' for i in range(1,6)}
            if expected-found:failures.append('active signal lenses incomplete')
        meta=json.loads((pkg/'tracks/processed'/f'{track}.json').read_text())
        if len(meta.get('grid',[]))<10:failures.append('insufficient spawn grids')
    except (OSError,ValueError,ET.ParseError,KeyError) as exc:failures.append(str(exc))
    return {'track':track,'status':'PASS' if not failures else 'FAIL','issues':failures}

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[2]);p.add_argument('--track',choices=(*TRACKS,'all'),default='all');args=p.parse_args()
    tracks=TRACKS if args.track=='all' else (args.track,)
    reports=[check(args.root,t) for t in tracks]
    for x in reports:print(f"{x['track']:13} {x['status']} {'; '.join(x['issues'])}")
    print('NOTE: static resource checks only. Visual on/off transitions must be checked in Gazebo.')
    return 0 if all(r['status']=='PASS' for r in reports) else 2
if __name__=='__main__':raise SystemExit(main())
