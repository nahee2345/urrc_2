#!/usr/bin/env python3
"""Package a team folder, src folder, or a single ROS package for intake."""
import argparse
import tempfile
import zipfile
from pathlib import Path
from submission import normalize


def main():
    p=argparse.ArgumentParser()
    p.add_argument('source',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('출력 파일이 이미 있습니다. 다른 파일명을 사용하세요.')
    with tempfile.TemporaryDirectory(prefix='urrc_entry_') as d:
        stage=Path(d)/'entry'
        eid=normalize(a.source,stage)
        a.output.parent.mkdir(parents=True,exist_ok=True)
        with zipfile.ZipFile(a.output,'w',compression=zipfile.ZIP_DEFLATED) as z:
            for f in sorted(stage.rglob('*')):
                if f.is_file():z.write(f,f.relative_to(stage))
    print(f'{eid}: {a.output.resolve()}')

if __name__=='__main__':main()
