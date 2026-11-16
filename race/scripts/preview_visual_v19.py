#!/usr/bin/env python3
"""Monza-standard visuals + official atomic five-red preview on all six tracks.

No participant sources, race results, road geometry or physical collisions edited.
"""
from __future__ import annotations
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

TRACKS = ('monza','spa','monaco','cota','hungaroring','baku')


def ensure_assets(root: Path, track: str) -> None:
    """Prepare v10.8 textured asphalt, offroad ground and identical light overlays.

    The standalone v17 preview missed this preparation.  It is intentionally
    performed only if any of the idempotent generated assets are missing.
    """
    if track not in TRACKS:
        raise ValueError('허용하지 않는 맵: ' + track)
    package = root / 'src/urrc_track_gazebo'
    worlds = package / 'worlds'
    model_mesh = package / 'models' / f'urrc_{track}' / 'meshes'
    race = worlds / f'{track}_race.sdf'
    if not race.is_file():
        raise FileNotFoundError(f'경기 맵 없음: {race}')
    rendered = (worlds/f'{track}_signal_active.sdf',
                model_mesh/'road_textured.obj',
                model_mesh/'asphalt_ground.obj',
                model_mesh/'offroad.png')
    source = race.read_text(encoding='utf-8')
    if any(not f.is_file() for f in rendered):
        sys.path.insert(0, str(package))
        from urrc_tracks.v10_assets import enhance_track
        print(f'[v19 VISUAL] {track}: Monza 기준 텍스처·5등 신호 리소스 준비', flush=True)
        enhance_track(package, track)
    else:
        print(f'[v19 VISUAL] {track}: 기존 텍스처·신호 리소스 확인', flush=True)
    for path in rendered:
        if not path.is_file():
            raise RuntimeError(f'{track}: 필수 시각 리소스 생성 실패: {path}')
    verify_gantry(package,track)


def verify_gantry(package: Path, track: str) -> None:
    """Standardise lamp geometry/materials; supports differ with lane width."""
    def lenses(name):
        world=ET.parse(package/'worlds'/f'{name}_race.sdf').getroot()
        gate=world.find("./world/model[@name='race_start_gantry']")
        if gate is None:
            raise ValueError(f'{name}: 출발 신호등 지지대 없음')
        link=gate.find("./link[@name='gantry_visuals']")
        if link is None:
            raise ValueError(f'{name}: 신호등 시각 링크 없음')
        out=[]
        for i in range(1,6):
            lens=link.find(f"./visual[@name='red_lens_{i}']")
            if lens is None:
                raise ValueError(f'{name}: 빨간 렌즈 {i} 없음')
            out.append((lens.findtext('pose'),lens.findtext('geometry/box/size'),
                        lens.findtext('material/ambient'),lens.findtext('material/diffuse'),
                        lens.findtext('material/emissive')))
        return out
    reference=lenses('monza')
    target=lenses(track)
    # Monza may not have been enhanced yet, while a different track already was.
    # Lens positions and dimensions are map-independent; its inactive material
    # naturally differs before/after v10 enhancement. Do not confuse this with
    # an actual signal hardware mismatch.
    if [row[:2] for row in reference] != [row[:2] for row in target]:
        raise ValueError(f'{track}: Monza와 5등 신호 렌즈 위치/규격 불일치. 자동으로 월드 형상을 덮어쓰지 않습니다.')
    expected=('0.015 0.005 0.006 1','0.015 0.005 0.006 1','0 0 0 1')
    if any(row[2:]!=expected for row in target):
        raise ValueError(f'{track}: v10.8 준비 후 신호등 재질이 표준과 다릅니다')
    print(f'[v19 VISUAL] {track}: Monza 기준 5등 렌즈 규격·표준 재질 PASS',flush=True)


def preview_request(track, service, reqtype, payload, *, env, optional=False):
    """Send requests to the PREVIEW partition only, not an official race."""
    if track not in TRACKS:
        raise RuntimeError(f'지원하지 않는 맵: {track}')
    command=['gz','service','-s',f'/world/{track}/{service}',
             '--reqtype',reqtype,'--reptype','gz.msgs.Boolean','--timeout','5000',
             '--req',payload]
    try:
        result=subprocess.run(command,env=env,capture_output=True,text=True,timeout=8)
    except (OSError,subprocess.TimeoutExpired) as exc:
        if optional:return False
        raise RuntimeError(f'{track}/{service} 실행 오류: {exc}') from exc
    ok=result.returncode==0 and re.search(r'\bdata:\s*true\b',result.stdout) is not None
    if not ok and not optional:
        raise RuntimeError(f'{track}/{service} 요청 실패: {result.stdout[-180:]!r} {result.stderr[-180:]!r}')
    return ok


def wait_for_gazebo_service(track: str, env: dict, proc, timeout=35.) -> None:
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        if proc.poll() is not None:
            raise RuntimeError(f'Gazebo 미리보기 조기 종료: code={proc.returncode}')
        try:
            listed=subprocess.run(['gz','service','-l'],env=env,capture_output=True,
                                  text=True,timeout=4,check=False)
            if listed.returncode==0 and f'/world/{track}/visual_config' in listed.stdout:
                return
        except (OSError,subprocess.TimeoutExpired):
            pass
        time.sleep(.35)
    raise RuntimeError(f'{track}: 신호등 제어 서비스 35초 이내 미준비')


def preview_signal_demo(root: Path,track: str,env: dict,proc) -> None:
    """Exactly the official v10 five individual ON / one shared OFF sequence."""
    from atomic_signal_v10 import AtomicFiveRed
    wait_for_gazebo_service(track,env,proc)
    from race_readiness_v20 import wait_for_visible_map
    wait_for_visible_map(track,env,stage='preview',world_process=proc)
    def request(t,s,r,p,**kw):
        return preview_request(t,s,r,p,env=env,**kw)
    gate=AtomicFiveRed(root,track,request=request)
    print(f'[v19 SIGNAL] {track}: 1~5등 순차 점등 시작',flush=True)
    try:
        gate.start()
        for i in range(1,6):
            gate.red(i)
            print(f'[v19 SIGNAL] RED {i}/5',flush=True)
            time.sleep(1.0)
        time.sleep(1.0)
        gate.all_off()
        print('[v19 SIGNAL] 동시 소등 PASS (단일 remove 요청)',flush=True)
    except BaseException:
        gate.cleanup_on_error()
        raise


def stop_preview(proc) -> None:
    """Signal the gz Ruby launcher once; avoid SIGINT to its entire process group.

    Gazebo's Ruby wrapper forwards SIGINT to its child itself. Group-wide SIGINT
    may cause duplicate kills and Errno::ESRCH. Escalate only on timeout.
    """
    if proc is None or proc.poll() is not None:return
    try:proc.send_signal(signal.SIGINT)
    except ProcessLookupError:return
    try:
        proc.wait(timeout=8)
        return
    except subprocess.TimeoutExpired:pass
    for sig in (signal.SIGTERM,signal.SIGKILL):
        try:os.killpg(proc.pid,sig)
        except ProcessLookupError:return
        try:
            proc.wait(timeout=3)
            return
        except subprocess.TimeoutExpired:continue
