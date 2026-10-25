#!/usr/bin/env python3
"""URRC practice v6: in-simulator continuous motion, no service polling."""
import argparse
import csv
import json
import math
import os
import random
import re
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'src/urrc_track_gazebo'
CSV = PACKAGE / 'tracks/processed/monza_centerline.csv'
WORLD_WITH_LIGHTS = PACKAGE / 'worlds/monza_race.sdf'
WORLD_WITHOUT_LIGHTS = PACKAGE / 'worlds/monza.sdf'
WORLD = WORLD_WITH_LIGHTS  # Backwards-compatible constant
GRID_META = PACKAGE / 'tracks/processed/monza.json'
NATIVE = ROOT / 'practice/native'


def read_track():
    with CSV.open(newline='') as f:
        rows = list(csv.DictReader(f))
    p = [(float(r['x_m']), float(r['y_m']), float(r['z_m'])) for r in rows]
    if len(p) < 5:
        raise RuntimeError('트랙 중심선 데이터가 부족합니다.')
    length = sum(math.dist(a[:2], b[:2]) for a, b in zip(p, p[1:] + p[:1]))
    return p, length


def sample(points, distance):
    ds = []
    length = 0.
    for a, b in zip(points, points[1:] + points[:1]):
        d = math.dist(a[:2], b[:2])
        ds.append(d)
        length += d
    s = distance % length
    for i, d in enumerate(ds):
        if s <= d:
            a, b = points[i], points[(i+1) % len(points)]
            f = s / d
            return tuple(a[k] + (b[k]-a[k]) * f for k in range(3))
        s -= d
    return points[0]


def car_model(name, color, grid, speed, lateral, points, plugin_path, start_gate_file=None):
    start = -float(grid["distance_behind_start_m"])
    # The three-sided painted white box is defined in geometry.py::grid_markings.
    # Its centre is track.point(parameter) + track.normal(parameter) * offset,
    # which is stored precisely as x/y/road_z/yaw in the generated monza.json.
    # Never apply a lateral offset to the spawn pose.
    x, y, road_z, yaw = (float(grid['x']), float(grid['y']),
                         float(grid['road_z']), float(grid['yaw']))
    model = ET.Element('model', {'name': name})
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x:.8f} {y:.8f} {road_z + .09:.8f} 0 0 {yaw:.8f}'
    link = ET.SubElement(model, 'link', {'name': 'base_link'})
    def shape(name, geometry, pose, material=None):
        for kind in ('collision', 'visual'):
            node = ET.SubElement(link, kind, {'name': name + '_' + kind})
            ET.SubElement(node, 'pose').text = pose
            geo = ET.SubElement(node, 'geometry')
            if geometry[0] == 'box':
                ET.SubElement(ET.SubElement(geo, 'box'), 'size').text = geometry[1]
            else:
                cyl = ET.SubElement(geo, 'cylinder')
                ET.SubElement(cyl, 'radius').text = str(geometry[1])
                ET.SubElement(cyl, 'length').text = str(geometry[2])
            if kind == 'visual' and material:
                mat = ET.SubElement(node, 'material')
                ET.SubElement(mat, 'ambient').text = material
                ET.SubElement(mat, 'diffuse').text = material
    # Exact bounding dimensions x=0.55, y=0.25, z=0.18.
    # z upper limit +0.09 chassis; lower -0.09 wheel.
    shape('chassis', ('box', '.55 .19 .14'), '0 0 .02 0 0 0', color + ' 1')
    for axle, px in [('front', .19), ('rear', -.19)]:
        for side, py in [('left', .105), ('right', -.105)]:
            shape(axle + '_' + side, ('cylinder', .045, .04),
                  f'{px} {py} -.045 1.57079632679 0 0', '.065 .065 .070 1')
    plugin = ET.SubElement(model, 'plugin', {
        'filename': str(plugin_path), 'name': 'urrc::PracticeMotion'})
    for key, value in [('centerline_csv', str(CSV.resolve())), ('speed_mps', speed),
                       ('start_m', start), ('lateral_m', lateral),
                       ('spawn_x', x), ('spawn_y', y), ('spawn_road_z', road_z),
                       ('spawn_yaw', yaw)]:
        ET.SubElement(plugin, key).text = str(value)
    if start_gate_file is not None:
        ET.SubElement(plugin, 'start_gate_file').text = str(start_gate_file)
    return model


def grid_start_positions():
    """Return unmodified center poses of the official painted grid boxes."""
    meta = json.loads(GRID_META.read_text())
    grid = {int(item['position']): item for item in meta['grid']}
    last = max(grid)
    if 2 not in grid or last < 3:
        raise RuntimeError('그리드 2번 또는 마지막 그리드가 없습니다.')
    for slot in (2, last):
        for key in ('x', 'y', 'road_z', 'yaw', 'distance_behind_start_m'):
            if key not in grid[slot] or not math.isfinite(float(grid[slot][key])):
                raise RuntimeError(f'그리드 {slot}의 {key} 좌표가 없습니다.')
    return [(grid[2], 2), (grid[last], last)]


def generate_world(output, cars, speeds, plugin_path, *, lights=True, start_gate_file=None):
    pts, distance = read_track()
    source_world = WORLD_WITH_LIGHTS if lights else WORLD_WITHOUT_LIGHTS
    if not source_world.is_file():
        raise RuntimeError(f'몬자 월드 파일이 없습니다: {source_world}')
    root = ET.parse(source_world).getroot()
    world = root.find('world')
    if world is None or world.get('name') != 'monza':
        raise RuntimeError('몬자 월드 구조가 올바르지 않습니다.')
    starts = grid_start_positions()
    # Separate sides of the track for predictable non-interfering practice traffic.
    # Faster B follows from the final grid. No teleport or overtaking claim.
    specs = [('urrc_practice_a', '.10 .35 .95', speeds[0], starts[0][0], -.4),
             ('urrc_practice_b', '1 .55 .08', speeds[1], starts[1][0], .4)]
    for name, color, speed, grid, lane in specs[:cars]:
        world.append(car_model(name, color, grid, speed, lane, pts, plugin_path, start_gate_file))
    ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
    return distance


def build_plugin():
    if not shutil.which('cmake'):
        raise RuntimeError('cmake가 없습니다: sudo apt install cmake g++ libgz-sim8-dev')
    build = ROOT / '.practice_native_build'
    lib = build / 'liburrc_practice_motion.so'
    subprocess.run(['cmake', '-S', str(NATIVE), '-B', str(build), '-DCMAKE_BUILD_TYPE=Release'], check=True)
    subprocess.run(['cmake', '--build', str(build), '-j2'], check=True)
    if not lib.is_file():
        raise RuntimeError('Gazebo 플러그인 라이브러리 생성 실패')
    return lib.resolve()



# Light sequence controls the existing five material-emissive lenses on the
# track gantry. A start-gate file is created ONLY after five verified off calls.
def _lens_payload(index, on):
    emission = 'r: 1 g: 0.008 b: 0.008 a: 1' if on else 'r: 0 g: 0 b: 0 a: 1'
    return (f'name: "red_lens_{index+1}" parent_name: "gantry_visuals" '
            'material { ambient { r: 0.12 g: 0.006 b: 0.008 a: 1 } '
            'diffuse { r: 0.12 g: 0.006 b: 0.008 a: 1 } '
            'specular { r: 0.02 g: 0.02 b: 0.02 a: 1 } '
            f'emissive {{ {emission} }} }}')


def _set_lens(index, on, env):
    cmd = ['gz', 'service', '-s', '/world/monza/visual_config', '--reqtype', 'gz.msgs.Visual',
           '--reptype', 'gz.msgs.Boolean', '--timeout', '1600', '--req', _lens_payload(index, on)]
    res = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=4)
    if res.returncode != 0 or not re.search(r'data:\s*true', res.stdout):
        raise RuntimeError(f'신호등 {index+1}번 제어 실패: {(res.stdout + res.stderr)[-300:]}')


def _set_all_lenses(on, env):
    with ThreadPoolExecutor(max_workers=5) as executor:
        list(executor.map(lambda i: _set_lens(i, on, env), range(5)))


def _wait_visual_service(proc, env, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            raise RuntimeError('Gazebo가 신호등 준비 이전에 종료됐습니다.')
        try:
            result = subprocess.run(['gz','service','-l'], env=env, capture_output=True, text=True, timeout=4)
            if '/world/monza/visual_config' in result.stdout.splitlines():
                return
        except subprocess.TimeoutExpired:
            pass
        time.sleep(.2)
    raise RuntimeError('Gazebo 신호등 서비스 준비 시간 초과')


def _random_intervals(rng):
    # Strictly positive, non-uniform, unpredictable 5-step lights-out timeline.
    # Reserve time for RPC latency: randomized nominal duration 2.1–3.6 s.
    total = rng.uniform(2.1, 3.6)
    raw = [rng.uniform(.1, 1.5) for _ in range(5)]
    return [total * v / sum(raw) for v in raw]


def _light_sequence(proc, env, start_file):
    # Fail closed: if any light command fails cars must remain stopped.
    _wait_visual_service(proc, env)
    _set_all_lenses(True, env)
    print('[URRC] 빨간불 5개 점등 완료; 불규칙한 소등 시퀀스 시작', flush=True)
    rng = random.SystemRandom()
    elapsed_start = time.monotonic()
    try:
        for n, delay in zip(range(4, -1, -1), _random_intervals(rng)):
            time.sleep(delay)
            if proc.poll() is not None:
                raise RuntimeError('신호등 도중 Gazebo가 종료됐습니다.')
            _set_lens(n, False, env)
            print(f'[URRC] 빨간불 {n+1}번 소등 (남은 불 {n}개)',flush=True)
        actual = time.monotonic() - elapsed_start
        if actual > 6.0:
            print(f'[URRC WARNING] 요청 처리 지연으로 6초 초과 ({actual:.2f}초)',flush=True)
        start_file.write_text('GO\n')
        print('[URRC] 모든 빨간불 소등 확인; 차량 출발 허용',flush=True)
    except Exception:
        # Attempt to restore five dark lenses, do not release vehicles.
        try: _set_all_lenses(False, env)
        except Exception: pass
        raise


def run():
    ap = argparse.ArgumentParser(description='몬자 연습: Gazebo 내부 매 시뮬레이션 주기 차량 이동')
    ap.add_argument('--cars', choices=(0, 1, 2), type=int, default=2)
    ap.add_argument('--speed-a', type=float, default=.7)
    ap.add_argument('--speed-b', type=float, default=1.4)
    ap.add_argument('--view', choices=('grid', 'overview'), default='grid')
    light_group = ap.add_mutually_exclusive_group()
    light_group.add_argument('--lights', dest='lights', action='store_true', help='한 번 빨간불 5개 점등→불규칙 소등 후 출발 (기본값)')
    light_group.add_argument('--no-lights', dest='lights', action='store_false', help='신호등 점등·소등 시퀀스 없이 즉시 주행')
    ap.set_defaults(lights=True)
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()
    if not all(math.isfinite(v) and .05 <= v <= 3 for v in (args.speed_a, args.speed_b)):
        ap.error('속도는 0.05~3.0m/s 여야 합니다.')
    if not (WORLD_WITH_LIGHTS if args.lights else WORLD_WITHOUT_LIGHTS).is_file() or not CSV.is_file() or not GRID_META.is_file():
        ap.error('몬자 월드/centerline 파일이 없습니다.')
    if args.dry_run:
        with tempfile.TemporaryDirectory() as t:
            n = generate_world(Path(t) / 'world.sdf', args.cars,
                               (args.speed_a, args.speed_b), Path('/test/liburrc_practice_motion.so'), lights=args.lights)
        print(f'PASS: cars={args.cars} lights={args.lights} speedA={args.speed_a} speedB={args.speed_b} track={n:.2f}m grids=2,last')
        for grid, index in grid_start_positions()[:args.cars]:
            print(f'GRID {index} PAINTED BOX CENTER x={grid["x"]:.8f} y={grid["y"]:.8f} ground_z={grid["road_z"]:.8f} yaw={grid["yaw"]:.8f}')
        return 0
    if not shutil.which('gz'):
        ap.error('Gazebo gz 명령을 찾을 수 없습니다.')
    if args.cars:
        try:
            lib = build_plugin()
        except (RuntimeError, subprocess.CalledProcessError) as ex:
            print('[URRC ERROR] Gazebo C++ 플러그인 빌드 실패:', ex, file=sys.stderr)
            print('Ubuntu 24.04에서 sudo apt install cmake g++ libgz-sim8-dev 실행 후 재시도하세요.', file=sys.stderr)
            return 2
    else:
        lib = Path('/unused.so')
    with tempfile.TemporaryDirectory(prefix='urrc_practice_v6_') as t:
        world_file = Path(t) / 'practice.sdf'
        start_file = Path(t) / 'lights_out_go.flag'
        generate_world(world_file, args.cars, (args.speed_a, args.speed_b), lib, lights=True, start_gate_file=start_file if args.lights else None)
        env = os.environ.copy()
        env.setdefault('GZ_PARTITION', 'urrc_practice_' + str(os.getuid()))
        env['GZ_SIM_RESOURCE_PATH'] = str(PACKAGE / 'models') + os.pathsep + env.get('GZ_SIM_RESOURCE_PATH', '')
        env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(lib.parent) + os.pathsep + env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        cmd = ['gz', 'sim', '-r', '-v', '3', '--gui-config', str(PACKAGE/'worlds'/f'monza_{args.view}.gui.config'), str(world_file)]
        print('[URRC] 연습 차량=%d / 출발 신호 시퀀스=%s / A %.2f B %.2f m/s' % (args.cars, 'ON' if args.lights else 'OFF', args.speed_a, args.speed_b), flush=True)
        proc = subprocess.Popen(cmd, env=env, cwd=ROOT, start_new_session=True)
        errors = []
        def light_worker():
            try:
                _light_sequence(proc, env, start_file)
            except Exception as e:
                errors.append(e)
                print('[URRC SIGNAL ERROR]', e, file=sys.stderr, flush=True)
        light_thread = threading.Thread(target=light_worker, daemon=True) if args.lights else None
        if light_thread:
            light_thread.start()
        try:
            while proc.poll() is None:
                if errors:
                    os.killpg(proc.pid, signal.SIGINT)
                    proc.wait(timeout=8)
                    return 2
                time.sleep(.15)
            return proc.returncode
        except KeyboardInterrupt:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                return proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                return proc.wait(timeout=5)

if __name__ == '__main__':
    sys.exit(run())
