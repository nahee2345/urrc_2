"""Regression tests for selection, geometry provenance and portable resources."""
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from urrc_tracks import TRACKS
from urrc_tracks.source import Track

spec = importlib.util.spec_from_file_location("run_world", PACKAGE / "scripts/run_world.py")
runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)


def test_only_monza_is_available():
    assert TRACKS == ("monza",)
    assert runner.choose_track("monza") == "monza"


def test_unknown_track_rejected():
    with pytest.raises(ValueError): runner.choose_track("bad")


@pytest.mark.parametrize("name", TRACKS)
def test_scale_is_metric_and_source_vertices_are_preserved(name):
    t = Track(PACKAGE, name)
    assert t.scale == .2
    reconstructed = t.curve(t.raw_s)
    assert np.max(np.abs(reconstructed - t.raw_xy)) < 1e-9
    # 1/5 is applied once to source positions; no target-length stretching.
    report = json.loads((PACKAGE / "docs" / f"{name}_geometry_report.json").read_text())
    assert report["length_normalization_applied"] is False


@pytest.mark.parametrize("name", TRACKS)
def test_seam_position_and_tangent_are_continuous(name):
    t = Track(PACKAGE, name)
    eps = 1e-5
    assert np.linalg.norm(t.point(t.period-eps) - t.point(eps)) < 4*eps
    assert np.linalg.norm(t.tangent(t.period-eps) - t.tangent(eps)) < 1e-4
    assert abs(t.height(t.period-eps)-t.height(eps)) < eps
    assert abs(t.width(t.period-eps)-t.width(eps)) < eps


@pytest.mark.parametrize("name", TRACKS)
def test_gui_config_has_top_level_plugins_and_full_course_camera(name):
    for view in ["overview", "grid"]:
        xml = (PACKAGE / "worlds" / f"{name}_{view}.gui.config").read_text()
        # Gazebo's tinyxml2 accepts multiple top-level elements; stdlib needs
        # a synthetic wrapper only for this inspection, never in the saved file.
        root = ET.fromstring("<test>" + xml.split("?>", 1)[1] + "</test>")
        assert root.find("gui") is None
        scene = root.find("plugin[@filename='MinimalScene']")
        assert scene is not None
        assert float(scene.findtext("camera_clip/far")) >= 2000
        assert root.find("plugin[@filename='InteractiveViewControl']") is not None


@pytest.mark.parametrize("name", TRACKS)
def test_single_world_command_and_staggered_twenty_slots(name):
    command = runner.command_for(PACKAGE, name)
    assert len([arg for arg in command if arg.endswith(".sdf")]) == 1
    data = json.loads((PACKAGE / "tracks/processed" / f"{name}.json").read_text())
    slots = data["grid"]
    assert [s["position"] for s in slots] == list(range(1,21))
    assert np.allclose(np.diff([s["distance_behind_start_m"] for s in slots]), 1.6)
    positions = np.array([[s["x"], s["y"]] for s in slots])
    distances = np.linalg.norm(positions[:,None]-positions[None,:],axis=2) + np.eye(20)*1e6
    assert distances.min() > 1.0
