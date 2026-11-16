#!/usr/bin/env python3
"""Operator-verified rendered-map readiness gate (v20).

Gazebo transport cannot assert that the Qt / Ogre client has rendered pixels.
Automated service/topic checks MUST be followed by explicit visual approval.
No implicit auto-ready fallback; no team, map, or scoring mutations.
"""
from __future__ import annotations
import os
import subprocess
import sys
import time

WORLD_READY_TIMEOUT = 120.0
POLL_SECONDS = 0.5
STABLE_PROBES = 3


def _running(proc, what):
    if proc is not None and proc.poll() is not None:
        raise RuntimeError(f'{what} 종료: code={proc.poll()} (신호등 미작동)')


def _listed(command, env):
    try:
        result = subprocess.run(command, env=env, text=True, capture_output=True,
                                timeout=5, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return set(line.strip() for line in result.stdout.splitlines() if line.strip())


def _probe(track, env):
    services = _listed(['gz','service','-l'],env)
    topics = _listed(['gz','topic','-l'],env)
    required_services = {f'/world/{track}/visual_config', f'/world/{track}/control'}
    required_topics = {f'/world/{track}/stats'}
    missing=[]
    if services is None:
        missing.append('Gazebo 서비스 목록 조회 실패')
    else:
        missing.extend(sorted(required_services-services))
    if topics is None:
        missing.append('Gazebo 토픽 목록 조회 실패')
    else:
        missing.extend(sorted(required_topics-topics))
    return missing


def wait_for_visible_map(track, env, *, stage='preview', world_process=None,
                         bridge_process=None, timeout=WORLD_READY_TIMEOUT,
                         stdin=None, stdout=None, checker=None, sleeper=None,
                         monotonic=None):
    """Block red lights until transport is stable AND operator sees rendered GUI.

    `READY` is a deliberate operator act, not proof of a programmatic image check.
    Missing TTY / missing Gazebo transport makes the race fail closed.
    Test-only injection parameters cannot be set through CLI or environment.
    """
    import re
    if not re.fullmatch(r'[a-z0-9_]+', track):
        raise ValueError('유효하지 않은 Gazebo 트랙 이름')
    if not isinstance(env,dict) or not env.get('GZ_PARTITION'):
        raise RuntimeError('독립 Gazebo 파티션 미설정: 신호등 시작 금지')
    stdin = sys.stdin if stdin is None else stdin
    stdout = sys.stdout if stdout is None else stdout
    checker = _probe if checker is None else checker
    sleeper = time.sleep if sleeper is None else sleeper
    monotonic = time.monotonic if monotonic is None else monotonic
    if not hasattr(stdin,'isatty') or not stdin.isatty():
        raise RuntimeError('운영자 터미널(TTY)이 없습니다. GUI 확인 없이 신호등을 자동 시작하지 않습니다.')
    _running(world_process, 'Gazebo 서버/GUI 실행기')
    _running(bridge_process, '중앙 ROS 2 브리지')
    print(f'[v20 READY] {stage} / {track}: 월드 서비스·시간 토픽 준비 대기 (최대 {int(timeout)}초)',
          file=stdout,flush=True)
    start = monotonic()
    stable = 0
    last_missing=[]
    while monotonic()-start < timeout:
        _running(world_process, 'Gazebo 서버/GUI 실행기')
        _running(bridge_process, '중앙 ROS 2 브리지')
        last_missing=checker(track,env)
        stable = stable+1 if not last_missing else 0
        if stable>=STABLE_PROBES:
            break
        sleeper(POLL_SECONDS)
    else:
        raise RuntimeError('Gazebo 월드 준비 시간 초과: '+', '.join(last_missing)+'. 신호등 시작 안 함')
    print('[v20 READY] Gazebo 월드 자동 점검 PASS. GUI 흰 화면이 사라지고',
          file=stdout,flush=True)
    print('            도로·그리드·신호등이 모두 보이는지 직접 확인하십시오.',file=stdout,flush=True)
    print('            준비가 되었을 때만 READY를 입력하십시오. Ctrl+C는 안전 중단합니다.',
          file=stdout,flush=True)
    while True:
        _running(world_process,'Gazebo 서버/GUI 실행기')
        _running(bridge_process,'중앙 ROS 2 브리지')
        try:
            value=input('[v20 READY] 화면 확인 완료 -> READY : ') if stdin is sys.stdin else _readline(stdin,stdout)
        except EOFError as exc:
            raise RuntimeError('운영자 화면 확인 입력이 종료되어 신호등을 시작하지 않습니다.') from exc
        if value.strip()=='READY':
            break
        print('[v20 READY] 승인되지 않았습니다. 화면 확인 후 정확히 READY를 입력하십시오.',
              file=stdout,flush=True)
    _running(world_process, 'Gazebo 서버/GUI 실행기')
    _running(bridge_process, '중앙 ROS 2 브리지')
    last_missing=checker(track,env)
    if last_missing:
        raise RuntimeError('승인 직후 Gazebo 서비스/토픽 이상: '+', '.join(last_missing)+'; 신호등 미작동')
    print(f'[v20 READY] {stage} / {track}: 운영자 확인 완료, 신호등 시작 허용',
          file=stdout,flush=True)
    return True


def _readline(stdin,stdout):
    print('[v20 READY] 화면 확인 완료 -> READY : ',end='',file=stdout,flush=True)
    line=stdin.readline()
    if not line:
        raise EOFError('입력 종료')
    return line.rstrip('\r\n')
