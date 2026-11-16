#!/usr/bin/env python3
"""Race-only five-red start signal: sequential on, single-model removal at lights out.

No participant source, original track world or original SDF model is modified.
The five lit visuals exist only in the temporary model urrc_start_signal_active.
"""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

ACTIVE_MODEL = 'urrc_start_signal_active'
LIGHT_COUNT = 5
VALID_TRACKS = frozenset(('monza', 'spa', 'monaco', 'cota', 'hungaroring', 'baku'))


class SignalError(RuntimeError):
    pass


def gz_request(track: str, service: str, reqtype: str, payload: str, *, optional=False) -> bool:
    """Require a positive Boolean reply; preserve current race GZ_PARTITION."""
    if track not in VALID_TRACKS:
        raise SignalError(f'알 수 없는 맵: {track}')
    command = [
        'gz', 'service', '-s', f'/world/{track}/{service}',
        '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
        '--timeout', '5000', '--req', payload,
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=8)
    except (OSError, subprocess.TimeoutExpired) as exc:
        if optional:
            return False
        raise SignalError(f'{service} 서비스 실행 실패: {exc}') from exc
    success = result.returncode == 0 and re.search(r'\bdata:\s*true\b', result.stdout) is not None
    if not success and not optional:
        raise SignalError(f'{service} 서비스 거부/시간 초과: stdout={result.stdout[-250:]!r}; stderr={result.stderr[-250:]!r}')
    return success


def _generate_template(root: Path, track: str) -> Path:
    """Generate ONLY a standalone signal model in race/results if the original isn't available."""
    package = root / 'src' / 'urrc_track_gazebo'
    template = package / 'worlds' / f'{track}_signal_active.sdf'
    if template.is_file():
        return template
    world = package / 'worlds' / f'{track}_race.sdf'
    if not world.is_file():
        raise SignalError(f'경기 월드 파일 없음: {world}')
    world_root = ET.parse(world).getroot()
    gate = world_root.find('.//world/model[@name="race_start_gantry"]')
    if gate is None:
        raise SignalError(f'{track}: race_start_gantry가 없습니다.')
    sdf = ET.Element('sdf', version='1.9')
    model = ET.SubElement(sdf, 'model', name=ACTIVE_MODEL)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = gate.findtext('pose', '0 0 0 0 0 0')
    link = ET.SubElement(model, 'link', name='signal_active_visuals')
    for number in range(1, LIGHT_COUNT + 1):
        source = gate.find(f'./link[@name="gantry_visuals"]/visual[@name="red_lens_{number}"]')
        if source is None:
            raise SignalError(f'{track}: red_lens_{number}이 없습니다.')
        visual = deepcopy(source)
        visual.set('name', f'active_red_lens_{number}')
        # Same overlay transform used by the existing standalone practice signal.
        pose = [float(value) for value in visual.findtext('pose', '0 0 0 0 0 0').split()]
        if len(pose) != 6:
            raise SignalError(f'{track}: red_lens_{number} pose 형식 오류')
        pose[0] -= 0.020
        visual.find('pose').text = ' '.join(f'{v:.7f}' for v in pose)
        size = visual.find('geometry/box/size')
        if size is None:
            raise SignalError(f'{track}: red_lens_{number} box 형상 없음')
        size.text = '.008 .110 .120'
        mat = visual.find('material')
        if mat is None:
            mat = ET.SubElement(visual, 'material')
        for tag, value in (
            ('ambient', '0.015 0.005 0.006 1'),
            ('diffuse', '0.015 0.005 0.006 1'),
            ('emissive', '0 0 0 1'),
        ):
            child = mat.find(tag)
            if child is None:
                child = ET.SubElement(mat, tag)
            child.text = value
        link.append(visual)
    output = root / 'race' / 'results' / 'signal_models_v10' / f'{track}_signal_active.sdf'
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(sdf)
    temp = output.with_suffix('.sdf.tmp')
    ET.ElementTree(sdf).write(temp, encoding='utf-8', xml_declaration=True)
    temp.replace(output)
    return output


def verify_template(path: Path) -> None:
    """Catch stale/incomplete signal model before a live start."""
    root = ET.parse(path).getroot()
    model = root.find(f'./model[@name="{ACTIVE_MODEL}"]')
    if model is None:
        raise SignalError(f'단일 신호등 모델이 아닙니다: {path}')
    link = model.find('./link[@name="signal_active_visuals"]')
    names = {v.get('name') for v in link.findall('visual')} if link is not None else set()
    expected = {f'active_red_lens_{number}' for number in range(1, LIGHT_COUNT+1)}
    if not expected.issubset(names):
        raise SignalError(f'단일 신호등 렌즈 부족: {sorted(expected - names)}')


class AtomicFiveRed:
    def __init__(self, root: Path, track: str, *, request=gz_request, sleep=time.sleep):
        if track not in VALID_TRACKS:
            raise SignalError(f'알 수 없는 맵: {track}')
        self.track = track
        self.request = request
        self.sleep = sleep
        self.template = _generate_template(Path(root), track)
        verify_template(self.template)
        self.active = False
        self.next_red = 1

    def start(self):
        # The prior instance may remain after an interrupted test. Remove it first.
        self.request(self.track, 'remove', 'gz.msgs.Entity',
                     f'name: "{ACTIVE_MODEL}" type: 2', optional=True)
        self.sleep(.3)
        self.request(self.track, 'create', 'gz.msgs.EntityFactory',
                     'sdf_filename: ' + json.dumps(str(self.template)))
        self.active = True
        self.next_red = 1
        self.sleep(.3)
        print('[v10 SIGNAL] 단일 5등 모델 준비 완료', flush=True)

    def red(self, number: int):
        if not self.active or number != self.next_red or number > LIGHT_COUNT:
            raise SignalError('신호등 점등 순서/상태 오류')
        payload = (f'name: "active_red_lens_{number}" parent_name: "signal_active_visuals" '
                   'material { ambient { r: 0.12 g: 0.005 b: 0.006 a: 1 } '
                   'diffuse { r: 0.12 g: 0.006 b: 0.008 a: 1 } '
                   'specular { r: 0.02 g: 0.02 b: 0.02 a: 1 } '
                   'emissive { r: 1 g: 0.008 b: 0.008 a: 1 } }')
        self.request(self.track, 'visual_config', 'gz.msgs.Visual', payload)
        self.next_red += 1

    def all_off(self):
        if not self.active or self.next_red != LIGHT_COUNT+1:
            raise SignalError('빨간불 5개가 모두 점등되지 않았습니다.')
        # CRITICAL: single Gazebo entity remove request, not five visual requests.
        self.request(self.track, 'remove', 'gz.msgs.Entity',
                     f'name: "{ACTIVE_MODEL}" type: 2')
        self.active = False
        print('[v10 SIGNAL] LIGHTS OUT: 단일 remove 요청 성공', flush=True)

    def cleanup_on_error(self):
        if self.active:
            self.request(self.track, 'remove', 'gz.msgs.Entity',
                         f'name: "{ACTIVE_MODEL}" type: 2', optional=True)
            self.active = False
