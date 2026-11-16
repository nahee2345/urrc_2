#!/usr/bin/env python3
"""Fail-closed static submission policy check, NOT runtime DDS isolation."""
import argparse,json,re,sys
from pathlib import Path
SUSPECT=[r'/world/',r'/race/(?!start/)',r'/model/[^\s\"\']+/urrc_timing',r'/clock\s*[^#\n]*publish',r'ROS_DOMAIN_ID\s*=',r'GZ_PARTITION\s*=']
EXT={'.py','.sh','.yaml','.yml','.xml','.sdf','.urdf','.xacro','.json','.cpp','.hpp','.h','.txt'}
def scan(root):
 root=Path(root)
 if not root.is_dir():raise ValueError('참가 소스 경로가 없습니다')
 findings=[]
 for p in root.rglob('*'):
  if not p.is_file() or p.suffix not in EXT or p.stat().st_size>2_000_000:continue
  if any(x in p.parts for x in ('.git','build','install','log')):continue
  try:s=p.read_text(encoding='utf-8')
  except UnicodeError:continue
  for i,line in enumerate(s.splitlines(),1):
   if any(re.search(rx,line) for rx in SUSPECT):findings.append(dict(file=str(p.relative_to(root)),line=i,snippet=line.strip()[:150]))
 return findings

def main():
 p=argparse.ArgumentParser();p.add_argument('--root',required=True);p.add_argument('--report');a=p.parse_args()
 try:
  f=scan(a.root);r={'status':'REVIEW_REQUIRED' if f else 'NO_STATIC_FLAGS','note':'정적 검사만 수행. DDS 격리/악성 접근 차단 보장하지 않음','findings':f}
  if a.report:Path(a.report).write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
  print(json.dumps(r,ensure_ascii=False,indent=2));return 3 if f else 0
 except (OSError,ValueError) as e:print(str(e),file=sys.stderr);return 2
if __name__=='__main__':sys.exit(main())
