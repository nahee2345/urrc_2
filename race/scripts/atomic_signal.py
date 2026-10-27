#!/usr/bin/env python3
"""URRC v10.8-compatible start lights shared by RACE and PRACTICE.

The five independently controllable lit lenses live in ONE temporary Gazebo
model.  Sequence: random sequential ON x5, then ONE model-remove request for
lights-out. The original gantry and its five dark backing lenses stay present.
This module intentionally does not touch vehicle motion, rules, or race timing.
"""
from copy import deepcopy
import atexit
import json
import math
import os
from pathlib import Path
import random
import re
import shutil
import subprocess
import tempfile
import time
from xml.etree import ElementTree as ET

_ACTIVE = "urrc_start_signal_active"
_TEMP_FILES = []


def _call(track, service, reqtype, payload, *, env=None, missing_ok=False):
    """Boolean Gazebo RPC with explicit partition env for practice sessions."""
    cmd = ["gz", "service", "-s", f"/world/{track}/{service}",
           "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
           "--timeout", "5000", "--req", payload]
    try:
        result = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=8)
    except (OSError, subprocess.SubprocessError) as exc:
        if missing_ok:
            return False
        raise RuntimeError(f"Gazebo {service} failed: {exc}") from exc
    ok = result.returncode == 0 and re.search(r"data:\s*true", result.stdout)
    if not ok and not missing_ok:
        raise RuntimeError(f"Gazebo {service} rejected: "
                           f"{result.stdout[-400:]} {result.stderr[-400:]}")
    return bool(ok)


def _active_template(root, track):
    """Equivalent to v10.8 v10_assets._write_active_signal_template.

    Build from the *actual* gantry in this checkout, not hard-coded poses.
    Retain the resulting SDF file until shutdown: Gazebo can read it
    asynchronously after the service ACK.
    """
    world = Path(root) / "src" / "urrc_track_gazebo" / "worlds" / f"{track}_race.sdf"
    if not world.is_file():
        raise FileNotFoundError(f"Missing race gantry world: {world}")
    gate = ET.parse(world).find(".//model[@name='race_start_gantry']")
    if gate is None:
        raise RuntimeError(f"Missing race_start_gantry in {world}")
    xml = ET.Element("sdf", version="1.9")
    model = ET.SubElement(xml, "model", name=_ACTIVE)
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = gate.findtext("pose", "0 0 0 0 0 0")
    link = ET.SubElement(model, "link", name="signal_active_visuals")
    for i in range(1, 6):
        original = gate.find(f"link[@name='gantry_visuals']/visual[@name='red_lens_{i}']")
        if original is None or original.find("geometry/box/size") is None:
            raise RuntimeError(f"Expected five independent gantry lenses; missing {i}")
        visual = deepcopy(original)
        visual.set("name", f"active_red_lens_{i}")
        coords = [float(v) for v in (visual.findtext("pose") or "").split()]
        if len(coords) != 6 or not all(math.isfinite(x) for x in coords):
            raise RuntimeError(f"Bad lamp pose {i}")
        coords[0] -= 0.020  # same forward overlay offset as working v10.8
        visual.find("pose").text = " ".join(f"{v:.7f}" for v in coords)
        visual.find("geometry/box/size").text = ".008 .110 .120"
        material = visual.find("material")
        if material is None:
            material = ET.SubElement(visual, "material")
        for tag, value in (("ambient", "0.015 0.005 0.006 1"),
                           ("diffuse", "0.015 0.005 0.006 1"),
                           ("emissive", "0 0 0 1")):
            node = material.find(tag)
            if node is None:
                node = ET.SubElement(material, tag)
            node.text = value
        link.append(visual)
    with tempfile.NamedTemporaryFile(prefix="urrc-v108-signal-",
                                     suffix=".sdf", delete=False) as temp:
        filename = Path(temp.name)
    _TEMP_FILES.append(filename)
    ET.ElementTree(xml).write(filename, encoding="utf-8", xml_declaration=True)
    return filename


def _cleanup():
    for path in _TEMP_FILES:
        try:
            path.unlink(missing_ok=True)
        except OSError:
            pass


atexit.register(_cleanup)


def all_off(track, root, *, env=None, missing_ok=True):
    """Remove all five illuminated lenses in ONE Gazebo entity operation."""
    return _call(track, "remove", "gz.msgs.Entity",
                 f'name: "{_ACTIVE}" type: 2', env=env, missing_ok=missing_ok)


def _create(track, root, *, env=None):
    template = _active_template(root, track)
    # A single model contains the five geometries: identical v10.8 layout.
    _call(track, "create", "gz.msgs.EntityFactory",
          "sdf_filename: " + json.dumps(str(template)), env=env)
    # The create RPC can complete before the next render iteration.
    time.sleep(0.35)


def lamp(track, index, enabled, root, *, env=None):
    """Zero-based index, as expected by the original race_manager interface."""
    if not 0 <= index < 5:
        raise ValueError("Lamp index must be 0..4")
    if not enabled:
        raise ValueError("Use all_off() for synchronized lights-out")
    if index == 0:
        all_off(track, root, env=env)
        time.sleep(0.30)
        _create(track, root, env=env)
    req = (f'name: "active_red_lens_{index+1}" parent_name: "signal_active_visuals" '
           'material { ambient { r: 0.12 g: 0.005 b: 0.006 a: 1 } '
           'diffuse { r: 0.12 g: 0.006 b: 0.008 a: 1 } '
           'specular { r: 0.02 g: 0.02 b: 0.02 a: 1 } '
           'emissive { r: 1 g: 0.008 b: 0.008 a: 1 } }')
    _call(track, "visual_config", "gz.msgs.Visual", req, env=env)
    return time.monotonic()


def lights_all(track, root, on, *, env=None):
    if not on:
        return all_off(track, root, env=env)
    for i in range(5):
        lamp(track, i, True, root, env=env)


def schedule(seed=None):
    """Unpredictable 5 sequential ON transitions, total first-on->off 3..6 s."""
    rng = random.Random(seed) if seed is not None else random.SystemRandom()
    total = rng.uniform(3.35, 5.05)
    hold = rng.uniform(0.40, min(0.90, total - 2.0))
    gap_budget = total - hold
    weights = [rng.uniform(0.45, 2.5) for _ in range(4)]
    gaps = [0.35 + (gap_budget - 1.40) * w / sum(weights) for w in weights]
    assert all(g > 0 for g in gaps)
    return gaps, hold


def sequence(track, root, *, env=None, seed=None):
    """Never release a vehicle before single-request lights-out succeeds."""
    all_off(track, root, env=env)
    gaps, hold = schedule(seed)
    first_on = None
    try:
        for i in range(5):
            at = lamp(track, i, True, root, env=env)
            if first_on is None:
                first_on = at
            print(f"[URRC SIGNAL] RED {i+1}/5", flush=True)
            if i < 4:
                time.sleep(gaps[i])
        deadline = first_on + sum(gaps) + hold
        time.sleep(max(0, deadline - time.monotonic()))
        all_off(track, root, env=env, missing_ok=False)
        elapsed = time.monotonic() - first_on
        print(f"[URRC SIGNAL] LIGHTS OUT model_remove=single_request "
              f"first_on_to_off={elapsed:.3f}s", flush=True)
        if not 3 < elapsed < 6:
            print("[URRC SIGNAL WARN] Gazebo RPC delays exceeded 3..6s target", flush=True)
        return elapsed
    except BaseException:
        all_off(track, root, env=env)
        raise
