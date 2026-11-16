"""Host health guard for official runs (Linux /proc; stdlib only)."""
import json
import os
import shutil
import time
from datetime import datetime, timezone

class HostOverload(RuntimeError):
    pass

class HostGuard:
    def __init__(self, log, *, cpu_limit=98., sustained_s=15., min_mem_mb=512., min_disk_mb=200., sample_s=.5):
        self.log=os.fspath(log)
        self.cpu_limit=cpu_limit
        self.sustained_s=sustained_s
        self.min_mem_mb=min_mem_mb
        self.min_disk_mb=min_disk_mb
        self.sample_s=sample_s
        self.last_check=0.
        self.prev_cpu=None
        self.over_start=None
    @staticmethod
    def cpu_sample():
        with open('/proc/stat',encoding='ascii') as f:
            p=f.readline().split()
        if p[0]!='cpu':raise OSError('/proc/stat is invalid')
        v=list(map(int,p[1:]));total=sum(v); idle=v[3]+(v[4] if len(v)>4 else 0)
        return total,idle
    @staticmethod
    def mem_available_mb():
        with open('/proc/meminfo',encoding='ascii') as f:
            for line in f:
                if line.startswith('MemAvailable:'):
                    return int(line.split()[1])/1024.
        raise OSError('MemAvailable unavailable')
    def fail(self, code, details):
        entry={'time_utc':datetime.now(timezone.utc).isoformat(), 'code':code,'details':details}
        os.makedirs(os.path.dirname(os.path.abspath(self.log)),exist_ok=True)
        with open(self.log,'a',encoding='utf-8') as f:f.write(json.dumps(entry,ensure_ascii=False)+'\n')
        raise HostOverload(f'{code}: {details}; 로그: {self.log}')
    def check(self, now=None):
        now=time.monotonic() if now is None else now
        if now-self.last_check<self.sample_s:return
        self.last_check=now
        try:
            mem=self.mem_available_mb()
            free=shutil.disk_usage(os.path.dirname(os.path.abspath(self.log))).free/1048576.
            cpu=self.cpu_sample()
        except (OSError,ValueError) as exc:
            self.fail('HOST_METRIC_ERROR',str(exc))
        if mem<self.min_mem_mb:self.fail('HOST_LOW_MEMORY',f'available={mem:.0f} MiB (<{self.min_mem_mb:.0f})')
        if free<self.min_disk_mb:self.fail('HOST_LOW_DISK',f'free={free:.0f} MiB (<{self.min_disk_mb:.0f})')
        if self.prev_cpu:
            dt=cpu[0]-self.prev_cpu[0]; di=cpu[1]-self.prev_cpu[1]
            usage=100.*(1-di/dt) if dt>0 else 0.
            if usage>=self.cpu_limit:
                if self.over_start is None:self.over_start=now
                if now-self.over_start>=self.sustained_s:
                    self.fail('HOST_HIGH_CPU',f'cpu={usage:.1f}% sustained >= {self.sustained_s}s')
            else:self.over_start=None
        self.prev_cpu=cpu
