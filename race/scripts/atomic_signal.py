"""Single-entity lights-out for the URRC starting gantry.

Only the temporary illuminated-lens model is removed. The existing five
unlit gantry lenses remain, and no race vehicle or timing model is changed.
"""
from copy import deepcopy
import json
import random
import re
import subprocess
import tempfile
import time
from pathlib import Path
from xml.etree import ElementTree as ET

_ACTIVE = "urrc_start_signal_active"


def _call(track, service, reqtype, request, *, optional=False):
    result = subprocess.run(
        ["gz", "service", "-s", f"/world/{track}/{service}",
         "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
         "--timeout", "5000", "--req", request],
        capture_output=True, text=True, timeout=8)
    ok = result.returncode == 0 and re.search(r"data:\s*true", result.stdout)
    if not ok and not optional:
        raise RuntimeError(f"Signal {service} failed: {result.stdout} {result.stderr}")
    return bool(ok)


def _create(track, root):
    src = Path(root) / "src" / "urrc_track_gazebo" / "worlds" / f"{track}_race.sdf"
    gate = ET.parse(src).find(".//model[@name='race_start_gantry']")
    if gate is None:
        raise RuntimeError(f"Missing gantry in {src}")
    model = ET.Element("model", name=_ACTIVE)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = gate.findtext("pose")
    link = ET.SubElement(model, "link", name="signal_active_visuals")
    for i in range(1, 6):
        lens = gate.find(f"link[@name='gantry_visuals']/visual[@name='red_lens_{i}']")
        if lens is None or lens.find("geometry/box/size") is None:
            raise RuntimeError(f"Missing gantry lens {i}")
        active = deepcopy(lens)
        active.set("name", f"active_red_lens_{i}")
        pose = [float(x) for x in active.findtext("pose").split()]
        pose[0] -= 0.020
        active.find("pose").text = " ".join(str(v) for v in pose)
        active.find("geometry/box/size").text = ".008 .110 .120"
        mat = active.find("material")
        if mat is None:
            mat = ET.SubElement(active, "material")
        ET.SubElement(mat, "emissive").text = "0 0 0 1"
        link.append(active)
    root_xml = ET.Element("sdf", version="1.9")
    root_xml.append(model)
    with tempfile.TemporaryDirectory(prefix="urrc-signal-") as tmp:
        filename = Path(tmp) / "signal_active.sdf"
        ET.ElementTree(root_xml).write(filename, encoding="utf-8", xml_declaration=True)
        # Gazebo service must read the file before this context closes.
        _call(track, "create", "gz.msgs.EntityFactory",
              "sdf_filename: " + json.dumps(str(filename)))
    time.sleep(0.35)


def lamp(track, index, enabled, root):
    if not 0 <= index < 5:
        raise ValueError("Lamp index must be 0..4")
    if not enabled:
        raise ValueError("Use all_off for synchronized five-lamp removal")
    if index == 0:
        all_off(track, root)
        _create(track, root)
    req = (
        f'name: "active_red_lens_{index + 1}" parent_name: "signal_active_visuals" '
        'material { ambient { r: 0.12 g: 0.005 b: 0.006 a: 1 } '
        'diffuse { r: 0.12 g: 0.006 b: 0.008 a: 1 } '
        'specular { r: 0.02 g: 0.02 b: 0.02 a: 1 } '
        'emissive { r: 1 g: 0.008 b: 0.008 a: 1 } }'
    )
    _call(track, "visual_config", "gz.msgs.Visual", req)


def all_off(track, root):
    # One entity-removal request instead of five visual updates.
    _call(track, "remove", "gz.msgs.Entity",
          f'name: "{_ACTIVE}" type: 2', optional=True)


def lights_all(track, root, on):
    if not on:
        return all_off(track, root)
    for index in range(5):
        lamp(track, index, True, root)


def schedule():
    """Five one-by-one reds, all out 3..6s after the first red."""
    rng = random.SystemRandom()
    total = rng.uniform(3.35, 5.15)
    hold = rng.uniform(0.40, 0.85)
    remaining = total - hold
    weights = [rng.uniform(0.5, 2.2) for _ in range(4)]
    weighted = sum(weights)
    gaps = [0.36 + (remaining - 4 * 0.36) * w / weighted for w in weights]
    return gaps, hold


def sequence(track, root):
    all_off(track, root)
    gaps, hold = schedule()
    first_at = None
    for i in range(5):
        lamp(track, i, True, root)
        if first_at is None:
            first_at = time.monotonic()
        if i < 4:
            time.sleep(gaps[i])
    target = first_at + sum(gaps) + hold
    time.sleep(max(0, target - time.monotonic()))
    all_off(track, root)
