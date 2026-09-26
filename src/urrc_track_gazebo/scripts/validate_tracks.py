#!/usr/bin/env python3
"""Validate packaged geometry. Does not label static checks as Gazebo tests."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
from shapely.geometry import LineString, Polygon


def validate(package):
    reports = []
    names = ("monza",)
    for name in names:
        model = package / "models" / ("urrc_" + name)
        meta = json.loads((package / "tracks/processed" / f"{name}.json").read_text())
        report = json.loads((package / "docs" / f"{name}_geometry_report.json").read_text())
        a = np.loadtxt(package / "tracks/processed" / f"{name}_centerline.csv", delimiter=",", skiprows=1)
        checks = {"finite_centerline": bool(np.isfinite(a).all()), "scale_exactly_0_2": report["scale"] == .2,
                  "length_within_1_percent": abs(report["length_error_xy_pct"]) <= 1,
                  "no_length_distortion": report["length_normalization_applied"] is False,
                  "positive_internal_width": bool((a[:, 5] > 0).all()), "grid_count_20": len(meta["grid"]) == 20}
        line = LineString(np.vstack([a[:, 1:3], a[0, 1:3]]))
        road = Polygon(meta["road_polygon"]["exterior"], [meta["road_polygon"]["interior"]])
        checks["single_nonintersecting_centerline"] = line.is_simple and line.is_ring
        checks["road_closed_single_corridor"] = road.is_valid and len(road.interiors) == 1 and road.buffer(1e-6).covers(line)
        ds = np.linalg.norm(np.roll(a[:, 1:3], -1, axis=0) - a[:, 1:3], axis=1)
        dz = np.roll(a[:, 3], -1) - a[:, 3]
        checks["no_zero_length_samples"] = bool((ds > 1e-7).all())
        checks["grade_below_35_percent"] = bool(np.max(np.abs(dz / ds)) < .35)
        mesh_counts = {}
        for key in ["road", "wall_outer", "wall_inner"]:
            mesh = trimesh.load(model / "meshes" / f"{key}.obj", force="mesh", process=False)
            collision_file = model / "meshes" / f"{key}_collision.obj"
            collider = trimesh.load(collision_file, force="mesh", process=False)
            checks[key + "_collision_closed"] = bool(collider.is_watertight and collider.is_winding_consistent and collider.volume > 0)
            checks[key + "_collision_face_reduction"] = len(collider.faces) < len(mesh.faces)
            raw_obj = collision_file.read_text()
            vertex_count = sum(1 for line in raw_obj.splitlines() if line.startswith("v "))
            normal_count = sum(1 for line in raw_obj.splitlines() if line.startswith("vn "))
            checks[key + "_gazebo_normals"] = (
                vertex_count == normal_count == len(collider.vertices)
                and all("//" in line for line in raw_obj.splitlines() if line.startswith("f "))
            )
            checks[key + "_finite"] = bool(np.isfinite(mesh.vertices).all())
            checks[key + "_closed_solid"] = bool(mesh.is_watertight and mesh.is_winding_consistent and mesh.volume > 0)
            checks[key + "_nondegenerate"] = bool(np.min(mesh.area_faces) > 1e-12)
            checks[key + "_single_body"] = mesh.body_count == 1
            mesh_counts[key] = {"vertices": len(mesh.vertices), "faces": len(mesh.faces)}
            if key == "road":
                top = mesh.triangles[mesh.face_normals[:, 2] > .5]
                delta1 = top[:, 1, :2] - top[:, 0, :2]
                delta2 = top[:, 2, :2] - top[:, 0, :2]
                planar_area = np.sum(np.abs(delta1[:,0]*delta2[:,1] - delta1[:,1]*delta2[:,0])) / 2
                checks["road_top_area_matches_annulus"] = bool(np.isclose(planar_area, road.area, rtol=1e-6))
            if key.startswith("wall"):
                parts = [trimesh.load(model / "meshes" / f"{key}_{label}.obj", force="mesh", process=False) for label in ("red", "white")]
                checks[key + "_visual_faces_complete"] = sum(len(m.faces) for m in parts) == len(mesh.faces)
                checks[key + "_visual_area_matches_collision"] = bool(np.isclose(sum(m.area for m in parts), mesh.area, rtol=1e-7))
                # Exported prisms have a vertical pair at each XY vertex.
                xy, inverse = np.unique(np.round(mesh.vertices[:, :2], 7), axis=0, return_inverse=True)
                max_z = np.full(len(xy), -np.inf); min_z = np.full(len(xy), np.inf)
                np.maximum.at(max_z, inverse, mesh.vertices[:, 2]); np.minimum.at(min_z, inverse, mesh.vertices[:, 2])
                checks[key + "_height_0_8_plus_foot_overlap"] = bool(np.allclose(max_z - min_z, .81, atol=2e-7))
        markings = trimesh.load(model / "meshes/markings.obj", force="mesh", process=False)
        checks["only_start_line_and_20_grid_markings"] = len(markings.faces) == 122
        checks["markings_inside_road"] = all(road.buffer(.002).covers(Polygon(markings.vertices[t, :2])) for t in markings.faces)
        for xml in [model / "model.sdf", model / "model.config", package / "worlds" / f"{name}.sdf"]:
            root = ET.parse(xml).getroot()
            for uri in root.findall(".//uri"):
                value = uri.text.strip()
                if not value.startswith("model://"):
                    raise AssertionError("Unexpected external or absolute URI: " + value)
                target = package / "models" / value[len("model://"):]
                if target.is_dir():
                    target /= "model.sdf"
                if not target.is_file():
                    raise AssertionError("Missing resource: " + str(target))
        world = ET.parse(package / "worlds" / f"{name}.sdf").getroot()
        checks["one_track_per_world"] = len(world.findall(".//include")) == 1
        model_xml = ET.parse(model / "model.sdf").getroot()
        checks["markings_have_no_collision"] = all("marking" not in c.get("name", "") for c in model_xml.findall(".//collision"))
        failed = [k for k, v in checks.items() if not v]
        result = {"track": name, "static_geometry": "FAIL" if failed else "PASS", "checks": checks,
                  "mesh_counts": mesh_counts, "failed": failed,
                  "gazebo_runtime": "NOT_RUN_BY_THIS_VALIDATOR", "survey_accuracy": "UNVERIFIED"}
        reports.append(result)
        print(f"{name:12s} {result['static_geometry']}" + (": " + ", ".join(failed) if failed else ""), flush=True)
    return reports


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--package", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = validate(args.package.resolve())
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    sys.exit(0 if all(r["static_geometry"] == "PASS" for r in result) else 1)
