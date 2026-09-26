"""Build all portable assets from versioned local input data."""
import json
from pathlib import Path
import numpy as np
import trimesh
from shapely.geometry import Polygon
from .source import Track
from .geometry import make_road, solid_road, make_wall_polygon, grid_markings
from .export import write_obj, write_model, write_world, write_gui
from .preview import preview, overview, grid_preview


def build_one(package, name):
    track = Track(package, name)
    road, offset_report = make_road(track)
    model = package / "models" / ("urrc_" + name)
    meshes = model / "meshes"; meshes.mkdir(parents=True, exist_ok=True)
    v, f, top, n = solid_road(road, track)
    write_obj(meshes / "road.obj", v, f)
    cv, cf, _, _ = solid_road(road, track, collision=True)
    write_obj(meshes / "road_collision.obj", cv, cf)
    mesh = trimesh.Trimesh(v, f, process=False)
    if not mesh.is_watertight or not mesh.is_winding_consistent:
        raise ValueError(f"{name}: road solid is not watertight with consistent normals")
    mesh_report = {"road": {"vertices": len(v), "triangles": len(f), "watertight": True}}
    for key, ring in [("wall_outer", road.exterior), ("wall_inner", road.interiors[0])]:
        footprint = make_wall_polygon(ring, track.config["wall"]["thickness_m"], key == "wall_outer")
        height = track.config["wall"]["height_m"]
        wv, wf, _, _ = solid_road(footprint, track, top_offset=height, depth=height + .01)
        cwv, cwf, _, _ = solid_road(footprint, track, top_offset=height, depth=height + .01, collision=True)
        write_obj(meshes / f"{key}_collision.obj", cwv, cwf)
        color = (track.project(wv[wf].mean(axis=1)[:, :2]) / track.config["wall"]["color_block_m"]).astype(int) % 2
        wall = trimesh.Trimesh(wv, wf, process=False)
        if not wall.is_watertight or not wall.is_winding_consistent:
            raise ValueError(f"{name}: invalid closed wall")
        write_obj(meshes / f"{key}.obj", wv, wf)
        for index, label in [(0, "red"), (1, "white")]:
            write_obj(meshes / f"{key}_{label}.obj", wv, wf[color == index])
        mesh_report[key] = {"vertices": len(wv), "triangles": len(wf), "watertight": True,
                            "height_above_road_m": track.config["wall"]["height_m"]}
    pv, pf, slots = grid_markings(track)
    write_obj(meshes / "markings.obj", pv, pf)
    for tri in pf:
        if not road.buffer(.002).covers(Polygon(pv[tri, :2])):
            raise ValueError(f"{name}: marking outside driving surface")
    write_model(model, name)
    write_world(package / "worlds" / f"{name}.sdf", track)
    write_world(package / "worlds" / f"{name}_race.sdf", track, traffic_lights=True)
    for mode in ["overview", "grid"]:
        write_gui(package / "worlds" / f"{name}_{mode}.gui.config", track, mode == "overview")
    process = package / "tracks/processed"
    samples = track.samples
    yaw = np.arctan2(track.tangent(samples)[:, 1], track.tangent(samples)[:, 0])
    data = np.column_stack([samples, track.xy, track.z, yaw, track.widths])
    np.savetxt(process / f"{name}_centerline.csv", data, delimiter=",", header="source_parameter_m,x_m,y_m,z_m,yaw_rad,road_width_m", comments="", fmt="%.8f")
    metadata = {"name": name, "scale": track.scale, "coordinates": "local East/North/Up; start/finish XY = 0,0",
                "projection": track.crs_wkt, "start_finish": {"x": 0., "y": 0., "z": float(track.height(0)), "yaw": float(yaw[0])},
                "centerline_csv_closure": "last row connects to first row; no duplicate endpoint", "grid": slots,
                "road_polygon": {"exterior": list(road.exterior.coords), "interior": list(road.interiors[0].coords)}}
    (process / f"{name}.json").write_text(json.dumps(metadata, indent=2) + "\n")
    target = track.config["official_length_real_m"] * track.scale
    length3d = np.linalg.norm(np.diff(np.vstack([np.c_[track.xy, track.z], np.r_[track.xy[0], track.z[0]]]), axis=0), axis=1).sum()
    report = {"track": name, "target_length_m": target, "length_xy_m": track.centerline.length,
              "length_3d_m": float(length3d), "length_error_xy_pct": (track.centerline.length / target - 1) * 100,
              "scale": track.scale, "length_normalization_applied": False,
              "centerline_simple": track.centerline.is_simple, "road_single_hole": len(road.interiors) == 1,
              "road_valid": road.is_valid, "width_min_m": float(track.widths.min()), "width_max_m": float(track.widths.max()),
              "elevation_range_m": float(np.ptp(track.z)), "grid_count": len(slots), "meshes": mesh_report,
              "source_shape_deviation_m": track.centerline.hausdorff_distance(__import__('shapely').geometry.LineString(track.raw_xy - track.origin_xy)),
              "official_survey_accuracy": "UNVERIFIED: public outline, estimated widths, DEM-based elevation",
              "gazebo_harmonic_runtime": "NOT_RUN", "gazebo_jetty_runtime": "NOT_RUN", "offsets": offset_report}
    if abs(report["length_error_xy_pct"]) > 1:
        raise ValueError(f"{name}: >1% length error; do not distort XY to hide source error")
    (package / "docs" / f"{name}_geometry_report.json").write_text(json.dumps(report, indent=2) + "\n")
    preview(package / "docs/previews", track, road, slots)
    grid_preview(package / "docs/previews", track)
    print(f"{name:12s} {report['length_xy_m']:10.3f} m  {report['length_error_xy_pct']:+.3f}%  {len(v):,} road vertices", flush=True)
    return track, road


def build_all(package, names):
    generated = [build_one(package, name) for name in names]
    if len(generated) == 6:
        overview(package / "docs/previews", generated)
