from pathlib import Path
import csv
import importlib.util
import re
import xml.etree.ElementTree as ET

import yaml

PKG = Path(__file__).resolve().parents[1]
WORKSPACE = PKG.parents[1]
CIRCUITS = {"monza", "silverstone", "spa", "suzuka", "monaco"}


def test_five_worlds_exist_and_parse():
    worlds = {p.stem for p in (PKG / "worlds").glob("*.sdf")}
    assert worlds == CIRCUITS
    for name in CIRCUITS:
        root = ET.parse(PKG / "worlds" / f"{name}.sdf").getroot()
        assert root.find(f".//world[@name='{name}']") is not None


def test_no_pits_or_green_floor():
    forbidden = ("pit_lane", "pitstop", "paddock", "0.16 0.30 0.13")
    for world in (PKG / "worlds").glob("*.sdf"):
        text = world.read_text(encoding="utf-8").lower()
        assert all(token not in text for token in forbidden)
        assert "0.045 0.045 0.050 1" in text
        assert "0.055 0.055 0.060 1" in text


def test_yellow_cones_and_half_metre_spacing():
    with (PKG / "config" / "track_specs.yaml").open(encoding="utf-8") as stream:
        specs = yaml.safe_load(stream)
    assert specs["cones"] == {"color": "yellow", "height_m": 0.09, "spacing_m": 0.5}
    for world in (PKG / "worlds").glob("*.sdf"):
        text = world.read_text(encoding="utf-8")
        assert "1.0 0.82 0.0 1" in text
        assert "yellow_cone.stl" in text
        assert "<scale>1 1 1.500000</scale>" in text
        assert "<length>0.09000</length>" in text

    with (PKG / "config" / "track_validation.csv").open(encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        assert 0.49 <= float(row["left_cone_spacing_m"]) <= 0.51
        assert 0.49 <= float(row["right_cone_spacing_m"]) <= 0.51


def test_vehicle_track_and_steering_contract():
    specs = yaml.safe_load((PKG / "config" / "track_specs.yaml").read_text(encoding="utf-8"))
    assert specs["vehicle_envelope_m"] == {"max_length": 0.55, "max_width": 0.25, "max_height": 0.18}
    assert specs["track"]["width_m"] == 1.5
    assert specs["track"]["pit_lane"] is False

    rows = list(csv.DictReader((PKG / "config" / "track_validation.csv").open(encoding="utf-8")))
    assert {row["circuit"] for row in rows} == CIRCUITS
    assert all(float(row["estimated_max_steering_deg"]) < 30.0 for row in rows)
    assert all(float(row["minimum_radius_m"]) > 0.75 for row in rows)


def test_only_suzuka_has_intentional_centerline_crossing():
    generator_path = PKG / "scripts" / "generate_tracks.py"
    spec = importlib.util.spec_from_file_location("track_generator", generator_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def intersects(a, b, c, d):
        def orient(p, q, r):
            return (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0])
        return orient(a, b, c)*orient(a, b, d) < 0 and orient(c, d, a)*orient(c, d, b) < 0

    counts = {}
    for name, controls in module.TRACKS.items():
        points, _, _, _ = module.prepare_centerline(controls)
        count = 0
        for i in range(len(points)):
            for j in range(i+2, len(points)):
                if i == 0 and j == len(points)-1:
                    continue
                count += intersects(points[i], points[(i+1) % len(points)], points[j], points[(j+1) % len(points)])
        counts[name] = count
    assert counts == {"monza": 0, "silverstone": 0, "spa": 0, "suzuka": 1, "monaco": 0}


def test_launch_scripts_are_nounset_safe_and_default_to_monza():
    default_script = (WORKSPACE / "run_gazebo.sh").read_text(encoding="utf-8")
    random_script = (WORKSPACE / "run_random_competition.sh").read_text(encoding="utf-8")
    assert "monza.launch.py" in default_script
    for text in (default_script, random_script):
        assert "set +u" in text
        assert re.search(r"set\s+-[^\n]*u", text) is None
        assert "--packages-select urrc_track_gazebo" in text


def test_spawn_pose_for_every_circuit():
    poses = yaml.safe_load((PKG / "config" / "spawn_poses.yaml").read_text(encoding="utf-8"))["circuits"]
    assert set(poses) == CIRCUITS
    assert all(set(pose) == {"x", "y", "z", "yaw_rad"} for pose in poses.values())
