#!/usr/bin/env python3
"""Host sampling monitor during live race. CPU/RAM/disk are host metrics, not RTF."""
import argparse, datetime, json, os, shutil, time
from pathlib import Path

def mem():
 vals={}
 for line in Path('/proc/meminfo').read_text().splitlines():
  if ':' in line:
   k,v=line.split(':',1)
   if k in ('MemTotal','MemAvailable'):vals[k]=int(v.split()[0])*1024
 return vals

def cpu():
 line=Path('/proc/stat').read_text().splitlines()[0].split()[1:]
 nums=list(map(int,line[:8]));return (sum(nums),nums[3]+nums[4])

def sample(root):
 a=mem();disk=shutil.disk_usage(root);c=cpu()
 return {'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
         'mem_used_percent':round(100*(1-a['MemAvailable']/a['MemTotal']),2),
         'disk_free_gb':round(disk.free/1e9,2),'cpu_counters':c}

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--interval',type=float,default=2);p.add_argument('--duration',type=float,default=120);p.add_argument('--disk-root',default='.');a=p.parse_args()
 if a.interval<.2 or a.duration<=0:p.error('interval >= .2, duration > 0')
 path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True)
 end=time.monotonic()+a.duration;last=None
 try:
  with path.open('a') as f:
   while time.monotonic()<end:
    x=sample(a.disk_root);tot,idle=x.pop('cpu_counters')
    x['cpu_busy_percent']=round(100*(1-(idle-last[1])/(tot-last[0])),2) if last and tot>last[0] else None
    last=(tot,idle)
    x['warning']=x['mem_used_percent']>=90 or x['disk_free_gb']<2 or (x['cpu_busy_percent'] is not None and x['cpu_busy_percent']>=95)
    f.write(json.dumps(x)+'\n');f.flush();time.sleep(min(a.interval,max(0,end-time.monotonic())))
 except KeyboardInterrupt:pass
if __name__=='__main__':main()
