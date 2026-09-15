#!/usr/bin/env python3
"""Generate five recognizable, 1/10-scale F1 representative Gazebo circuits.

The waypoint sets follow the sequence of signature corner complexes on each real
circuit. They are simplified and enlarged until a conservative steering check
passes; they are not survey data.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

Point = Tuple[float, float]
Point3 = Tuple[float, float, float]

VEHICLE_MAX_LENGTH = 0.55
VEHICLE_MAX_WIDTH = 0.25
VEHICLE_MAX_HEIGHT = 0.18
WHEELBASE_FOR_VALIDATION = 0.33
MAX_ALLOWED_STEERING_DEG = 29.0
DESIGN_STEERING_DEG = 20.0
TRACK_WIDTH = 1.50
CONE_HEIGHT = VEHICLE_MAX_HEIGHT / 2.0
CONE_BASE_RADIUS = 0.022
CONE_COLLISION_RADIUS = 0.018
CONE_SPACING = 0.50
ROAD_SAMPLE_SPACING = 0.22
VALIDATION_SAMPLE_SPACING = 0.04
ROAD_THICKNESS = 0.012
FLOOR_RGB = "0.045 0.045 0.050 1"
ROAD_RGB = "0.055 0.055 0.060 1"

# Ordered signature complexes are retained by interpolation instead of being
# reduced to a generic low-harmonic loop.
TRACKS: Dict[str, Sequence[Point]] = {
    # Rettifilo, Curva Grande, Roggia, Lesmos, Ascari, Parabolica.
    "monza": [
        (10.0, -7.5), (10.0, -3.0), (10.0, 2.5), (10.0, 6.0),
        (9.0, 7.2), (7.7, 6.2), (6.3, 7.1),
        (-1.2, 7.5), (-3.8, 6.5), (-5.5, 4.5),
        (-6.1, 2.8), (-5.1, 1.7), (-6.2, 0.6),
        (-7.0, -1.0), (-6.5, -2.7), (-5.0, -3.8),
        (-3.3, -4.2), (-2.0, -3.4), (-0.7, -4.3), (0.8, -3.5),
        (2.3, -4.8), (5.0, -5.1), (8.0, -5.0),
        (9.0, -5.5), (9.5, -6.5), (9.4, -7.5), (9.2, -8.1),
        (9.5, -8.6), (10.0, -8.5),
    ],
    # Abbey/Farm, Village/Loop, Wellington, Luffield, Copse,
    # Maggotts-Becketts-Chapel, Hangar, Stowe, Vale/Club.
    "silverstone": [
        (-4.0, -7.0), (1.0, -7.0), (5.0, -6.7), (7.0, -5.4),
        (7.8, -3.5), (6.8, -1.8), (5.0, -1.0),
        (3.0, -1.4), (1.8, -2.8), (0.5, -3.8),
        (-3.0, -4.0), (-6.5, -3.7), (-8.0, -2.5),
        (-7.7, -0.8), (-6.0, 0.2), (-4.5, 0.8),
        (-4.2, 3.7), (-3.0, 5.8), (-0.5, 7.0), (2.0, 6.8),
        (3.7, 5.7), (4.7, 4.4), (3.6, 3.3), (4.8, 2.2),
        (3.9, 1.1), (5.2, 0.3), (8.0, -0.2), (11.0, -1.0),
        (12.0, -2.5), (11.0, -4.3), (8.8, -5.3),
        (7.0, -5.1), (6.0, -6.1), (4.7, -6.8),
        (1.5, -7.8), (-2.0, -8.4), (-4.5, -8.5), (-6.0, -8.0),
        (-6.8, -7.0), (-6.0, -6.4),
    ],
    # La Source, Eau Rouge/Raidillon, Kemmel, Les Combes, Bruxelles/Pouhon,
    # Fagnes, Stavelot/Blanchimont, Bus Stop.
    "spa": [
        (-7.0, -8.5), (-2.0, -8.5), (1.0, -8.5),
        (2.5, -7.5), (2.5, -6.0), (1.0, -5.2), (-1.0, -5.3),
        (-3.2, -4.0), (-4.3, -2.7), (-3.8, -1.3), (-2.5, 0.2),
        (0.5, 2.6), (4.0, 5.1), (8.0, 7.3), (11.0, 7.5),
        (12.3, 6.5), (11.7, 5.0), (10.1, 4.5), (9.0, 3.5),
        (9.6, 2.0), (8.7, 0.8), (7.2, 0.0),
        (6.6, -1.6), (7.8, -2.8), (9.5, -2.4),
        (11.2, -3.2), (11.0, -4.8), (9.2, -5.8),
        (5.5, -8.0), (2.0, -9.0), (-2.0, -9.3), (-5.0, -9.5),
        (-7.0, -9.5), (-9.0, -9.0), (-9.5, -8.0), (-8.5, -7.5),
        (-7.5, -8.0),
    ],
    # Turn 1/2, S Curves, Dunlop, Degners, Hairpin, Spoon, crossover back
    # straight, 130R, Casio Triangle. The later crossover pass is elevated.
    "suzuka": [
        (7.0, -6.0), (7.0, -2.0), (7.2, 1.0), (6.3, 3.0),
        (4.7, 4.0), (3.0, 3.3), (3.5, 2.0), (2.0, 1.0),
        (2.8, 0.0), (1.2, -0.9), (1.8, -2.0), (0.0, -2.7),
        (-2.0, -1.8), (-3.2, 0.0), (-4.0, 2.3),
        (-5.2, 1.7), (-5.8, 0.2), (-7.5, -0.8),
        (-8.2, -2.0), (-7.2, -3.0), (-5.5, -3.2),
        (-4.2, -4.3), (-4.8, -5.7), (-3.2, -6.8), (-1.2, -5.8),
        (1.0, -4.4), (3.0, -2.3), (5.0, -0.2), (7.0, 1.8),
        (9.0, 3.0), (10.8, 2.3), (11.5, 0.2), (11.0, -2.5),
        (10.2, -5.0), (9.0, -6.8), (8.0, -7.5), (7.0, -7.8),
        (6.3, -7.2), (6.2, -6.5),
    ],
    # Sainte Devote/Beau Rivage, Massenet/Casino/Mirabeau, Grand Hotel
    # hairpin, Portier/Tunnel, Nouvelle, Tabac/Swimming Pool, Rascasse/Noghes.
    "monaco": [
        (0.0, -7.0), (4.0, -7.0), (9.0, -6.3),
        (11.5, -4.8), (11.8, -2.0), (10.0, 1.3), (7.0, 2.5),
        (2.5, 3.5), (0.0, 4.5), (-3.0, 4.5), (-5.0, 3.2),
        (-6.0, 1.8), (-6.5, 0.4), (-6.0, -0.9), (-4.8, -1.6),
        (-3.4, -1.5), (-2.0, -0.9), (-0.5, -0.2), (1.5, 0.3),
        (3.5, 0.6), (6.0, 0.5), (8.5, 0.0), (10.0, -1.2),
        (11.0, -2.8), (10.8, -4.2), (9.8, -5.2), (8.8, -5.8),
        (7.5, -5.8), (6.2, -5.1), (5.2, -4.2), (4.2, -5.0),
        (3.2, -4.2), (2.2, -5.0), (0.0, -5.0), (-2.0, -5.0),
        (-3.8, -5.6), (-4.8, -6.6), (-4.1, -7.5), (-2.4, -7.3),
    ],
}


def periodic_bspline(points: Sequence[Point], samples_per_segment: int = 50) -> List[Point]:
    """C2-continuous closed cubic B-spline retaining the waypoint silhouette."""
    pts, out = list(points), []
    for i in range(len(pts)):
        support = (pts[(i-1) % len(pts)], pts[i], pts[(i+1) % len(pts)], pts[(i+2) % len(pts)])
        for j in range(samples_per_segment):
            t = j/samples_per_segment
            t2, t3 = t*t, t*t*t
            weights = ((-t3+3*t2-3*t+1)/6, (3*t3-6*t2+4)/6,
                       (-3*t3+3*t2+3*t+1)/6, t3/6)
            out.append(tuple(sum(weights[k]*support[k][axis] for k in range(4)) for axis in (0, 1)))
    return out


def cumulative_lengths_closed(points: Sequence[Tuple[float, ...]]) -> Tuple[List[float], float]:
    cum, total = [0.0], 0.0
    for i, p1 in enumerate(points):
        p2 = points[(i+1) % len(points)]
        total += math.sqrt(sum((p2[k]-p1[k])**2 for k in range(len(p1))))
        cum.append(total)
    return cum, total


def point_at_s(points: Sequence[Tuple[float, ...]], cum: Sequence[float], total: float, s: float) -> Tuple[float, ...]:
    s %= total
    lo, hi = 0, len(points)-1
    while lo <= hi:
        mid = (lo+hi)//2
        if cum[mid+1] < s:
            lo = mid+1
        elif cum[mid] > s:
            hi = mid-1
        else:
            seg = cum[mid+1]-cum[mid]
            t = 0.0 if seg < 1e-12 else (s-cum[mid])/seg
            p, q = points[mid], points[(mid+1) % len(points)]
            return tuple(p[k]+t*(q[k]-p[k]) for k in range(len(p)))
    return tuple(points[-1])


def resample_closed(points: Sequence[Tuple[float, ...]], spacing: float) -> List[Tuple[float, ...]]:
    cum, total = cumulative_lengths_closed(points)
    count = max(3, int(round(total/spacing)))
    step = total/count
    return [point_at_s(points, cum, total, i*step) for i in range(count)]


def estimated_max_steering(points: Sequence[Point], wheelbase: float) -> Tuple[float, float]:
    max_curvature = 0.0
    for i, p1 in enumerate(points):
        p0, p2 = points[(i-1) % len(points)], points[(i+1) % len(points)]
        a = math.hypot(p1[0]-p0[0], p1[1]-p0[1])
        b = math.hypot(p2[0]-p1[0], p2[1]-p1[1])
        c = math.hypot(p2[0]-p0[0], p2[1]-p0[1])
        area2 = abs((p1[0]-p0[0])*(p2[1]-p0[1])-(p1[1]-p0[1])*(p2[0]-p0[0]))
        if a*b*c > 1e-10:
            max_curvature = max(max_curvature, 2.0*area2/(a*b*c))
    return math.degrees(math.atan(wheelbase*max_curvature)), max_curvature


def scale_points(points: Sequence[Point], scale: float) -> List[Point]:
    return [(x*scale, y*scale) for x, y in points]


def prepare_centerline(control: Sequence[Point]) -> Tuple[List[Point], float, float, float]:
    base, scale = periodic_bspline(control), 1.0
    while True:
        dense = resample_closed(scale_points(base, scale), VALIDATION_SAMPLE_SPACING)
        steer, _ = estimated_max_steering(dense, WHEELBASE_FOR_VALIDATION)
        if steer <= DESIGN_STEERING_DEG:
            break
        scale *= 1.035
        if scale > 5.5:
            raise RuntimeError("Unable to meet steering design margin")
    road = resample_closed(scale_points(base, scale), ROAD_SAMPLE_SPACING)
    steer, curvature = estimated_max_steering(resample_closed(scale_points(base, scale), VALIDATION_SAMPLE_SPACING), WHEELBASE_FOR_VALIDATION)
    return road, scale, steer, math.inf if curvature <= 1e-12 else 1.0/curvature


def suzuka_elevations(center: Sequence[Point], scale: float) -> List[float]:
    cum, total = cumulative_lengths_closed(center)
    target = (3.0*scale, -2.3*scale)
    crossing_i = min(range(len(center)//2, len(center)), key=lambda i: math.hypot(center[i][0]-target[0], center[i][1]-target[1]))
    center_s, half_span = cum[crossing_i], max(8.0, 6.0*scale)
    result = []
    for s in cum[:-1]:
        d = min(abs(s-center_s), total-abs(s-center_s))
        result.append(0.68*0.5*(1.0+math.cos(math.pi*d/half_span)) if d < half_span else 0.0)
    return result


def tangent_normal(points: Sequence[Point3], i: int) -> Tuple[float, float]:
    prev, nxt = points[(i-1) % len(points)], points[(i+1) % len(points)]
    tx, ty = nxt[0]-prev[0], nxt[1]-prev[1]
    length = math.hypot(tx, ty) or 1.0
    return -ty/length, tx/length


def boundary_points(center: Sequence[Point3], side: float) -> List[Point3]:
    result = []
    for i, (x, y, z) in enumerate(center):
        nx, ny = tangent_normal(center, i)
        result.append((x+side*TRACK_WIDTH*nx/2, y+side*TRACK_WIDTH*ny/2, z))
    return result


def segment_pose(p1: Point3, p2: Point3) -> Tuple[float, float, float, float, float, float]:
    dx, dy, dz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
    horizontal = math.hypot(dx, dy)
    return ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2,
            math.sqrt(horizontal*horizontal+dz*dz), -math.atan2(dz, horizontal), math.atan2(dy, dx))


def sdf_header(name: str) -> List[str]:
    return [
        "<?xml version='1.0'?>", "<sdf version='1.10'>", f"  <world name='{name}'>",
        "    <physics name='1ms' type='ignored'><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>",
        "    <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>",
        "    <plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>",
        "    <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>",
        "    <scene><ambient>0.42 0.42 0.44 1</ambient><background>0.055 0.055 0.060 1</background><shadows>true</shadows></scene>",
        "    <light name='sun' type='directional'><pose>0 0 30 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -1</direction></light>",
        "    <model name='ground'><static>true</static><link name='ground_link'>",
        "      <collision name='ground_collision'><geometry><plane><normal>0 0 1</normal><size>160 160</size></plane></geometry></collision>",
        f"      <visual name='ground_visual'><geometry><plane><normal>0 0 1</normal><size>160 160</size></plane></geometry><material><ambient>{FLOOR_RGB}</ambient><diffuse>{FLOOR_RGB}</diffuse></material></visual>",
        "    </link></model>",
    ]


def generate_world(name: str, center2: Sequence[Point], scale: float, out_file: Path) -> Dict[str, float]:
    elevations = suzuka_elevations(center2, scale) if name == "suzuka" else [0.0]*len(center2)
    center = [(p[0], p[1], elevations[i]) for i, p in enumerate(center2)]
    dense3 = resample_closed(center, 0.08)
    left = resample_closed(boundary_points(dense3, 1.0), CONE_SPACING)
    right = resample_closed(boundary_points(dense3, -1.0), CONE_SPACING)
    lines = sdf_header(name)

    lines += ["    <model name='road'><static>true</static><link name='road_link'>"]
    for i, p1 in enumerate(center):
        p2 = center[(i+1) % len(center)]
        x, y, z, length, pitch, yaw = segment_pose(p1, p2)
        pose_z = z+ROAD_THICKNESS/2
        lines.append(f"      <visual name='road_{i}'><pose>{x:.5f} {y:.5f} {pose_z:.5f} 0 {pitch:.7f} {yaw:.7f}</pose><geometry><box><size>{length+0.09:.5f} {TRACK_WIDTH:.5f} {ROAD_THICKNESS:.5f}</size></box></geometry><material><ambient>{ROAD_RGB}</ambient><diffuse>{ROAD_RGB}</diffuse><specular>0.04 0.04 0.04 1</specular></material></visual>")
        if name == "suzuka" and (p1[2] > 0.002 or p2[2] > 0.002):
            lines.append(f"      <collision name='bridge_road_{i}'><pose>{x:.5f} {y:.5f} {pose_z:.5f} 0 {pitch:.7f} {yaw:.7f}</pose><geometry><box><size>{length+0.09:.5f} {TRACK_WIDTH:.5f} {ROAD_THICKNESS:.5f}</size></box></geometry></collision>")
    lines += ["    </link></model>"]

    lines += ["    <model name='yellow_cones'><static>true</static><link name='cones_link'>"]
    cone_count = 0
    for side_name, boundary in (("left", left), ("right", right)):
        for i, (x, y, z) in enumerate(boundary):
            cone_count += 1
            cone_z = z+ROAD_THICKNESS+CONE_HEIGHT/2
            lines += [
                f"      <visual name='{side_name}_cone_{i}'><pose>{x:.5f} {y:.5f} {cone_z:.5f} 0 0 0</pose><geometry><mesh><uri>model://urrc_track_gazebo/meshes/yellow_cone.stl</uri><scale>1 1 {CONE_HEIGHT/0.06:.6f}</scale></mesh></geometry><material><ambient>1.0 0.78 0.0 1</ambient><diffuse>1.0 0.82 0.0 1</diffuse><specular>0.2 0.2 0.05 1</specular></material></visual>",
                f"      <collision name='{side_name}_cone_col_{i}'><pose>{x:.5f} {y:.5f} {cone_z:.5f} 0 0 0</pose><geometry><cylinder><radius>{CONE_COLLISION_RADIUS:.5f}</radius><length>{CONE_HEIGHT:.5f}</length></cylinder></geometry></collision>",
            ]
    lines += ["    </link></model>"]

    p0, p1 = center[0], center[1]
    yaw = math.atan2(p1[1]-p0[1], p1[0]-p0[0])
    lines += [
        "    <model name='start_line'><static>true</static><link name='start_line_link'>",
        f"      <visual name='start_bar'><pose>{p0[0]:.5f} {p0[1]:.5f} {p0[2]+ROAD_THICKNESS+0.002:.5f} 0 0 {yaw:.7f}</pose><geometry><box><size>0.08 {TRACK_WIDTH*0.92:.5f} 0.004</size></box></geometry><material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material></visual>",
        "    </link></model>", "  </world>", "</sdf>",
    ]
    out_file.write_text("\n".join(lines)+"\n", encoding="utf-8")

    _, lap_length = cumulative_lengths_closed(center)
    spacings = []
    for boundary in (left, right):
        _, boundary_length = cumulative_lengths_closed(boundary)
        spacings.append(boundary_length/len(boundary))
    return {
        "lap_length_m": lap_length, "cones_total": float(cone_count),
        "left_cone_spacing_m": spacings[0], "right_cone_spacing_m": spacings[1],
        "spawn_x": p0[0], "spawn_y": p0[1], "spawn_z": p0[2]+0.12,
        "spawn_yaw_rad": yaw, "max_elevation_m": max(elevations),
    }


def write_cone_stl(path: Path) -> None:
    sides, r, h, triangles = 16, CONE_BASE_RADIUS, 0.06, []
    for i in range(sides):
        a0, a1 = 2*math.pi*i/sides, 2*math.pi*(i+1)/sides
        p0, p1 = (r*math.cos(a0), r*math.sin(a0), 0.0), (r*math.cos(a1), r*math.sin(a1), 0.0)
        triangles += [(p0, p1, (0.0, 0.0, h)), ((0.0, 0.0, 0.0), p1, p0)]
    def normal(a: Point3, b: Point3, c: Point3) -> Point3:
        u, v = tuple(b[i]-a[i] for i in range(3)), tuple(c[i]-a[i] for i in range(3))
        n = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
        mag = math.sqrt(sum(x*x for x in n)) or 1.0
        return tuple(x/mag for x in n)
    out = ["solid yellow_cone"]
    for a, b, c in triangles:
        n = normal(a, b, c)
        out += [f"  facet normal {n[0]:.7f} {n[1]:.7f} {n[2]:.7f}", "    outer loop"]
        out += [f"      vertex {p[0]:.7f} {p[1]:.7f} {p[2]:.7f}" for p in (a, b, c)]
        out += ["    endloop", "  endfacet"]
    out.append("endsolid yellow_cone")
    path.write_text("\n".join(out)+"\n", encoding="ascii")


def write_preview(path: Path, prepared: Dict[str, Sequence[Point]]) -> None:
    panels = []
    for index, name in enumerate(("monza", "monaco", "silverstone", "spa", "suzuka")):
        pts = prepared[name]
        col, row = index % 3, index // 3
        ox, oy, width, height = 20+col*400, 45+row*330, 360, 275
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        factor = 235/max(max(xs)-min(xs), max(ys)-min(ys))
        coords = " ".join(f"{ox+width/2+(x-(max(xs)+min(xs))/2)*factor:.1f},{oy+height/2-(y-(max(ys)+min(ys))/2)*factor:.1f}" for x, y in pts)
        panels += [
            f"<text x='{ox}' y='{oy-12}' fill='#f5f5f5' font-size='19' font-family='sans-serif'>{name.title()}</text>",
            f"<polyline points='{coords} {coords.split()[0]}' fill='none' stroke='#0e0e10' stroke-width='30' stroke-linejoin='round'/>",
            f"<polyline points='{coords} {coords.split()[0]}' fill='none' stroke='#e7b900' stroke-width='2' stroke-dasharray='2 7'/>",
        ]
    svg = ["<svg xmlns='http://www.w3.org/2000/svg' width='1220' height='700' viewBox='0 0 1220 700'>", "<rect width='100%' height='100%' fill='#202024'/>"]+panels+["</svg>"]
    path.write_text("\n".join(svg)+"\n", encoding="utf-8")


def main() -> None:
    pkg = Path(__file__).resolve().parents[1]
    worlds, meshes, config, docs = pkg/"worlds", pkg/"meshes", pkg/"config", pkg/"docs"
    for directory in (worlds, meshes, config, docs):
        directory.mkdir(exist_ok=True)
    write_cone_stl(meshes/"yellow_cone.stl")

    rows, prepared = [], {}
    spawn_lines = ["# Vehicle model is intentionally not included. Spawn your own car at these poses.", "circuits:"]
    for name, control in TRACKS.items():
        center, scale, max_steer, min_radius = prepare_centerline(control)
        prepared[name] = center
        meta = generate_world(name, center, scale, worlds/f"{name}.sdf")
        meta.update(circuit=name, geometry_scale=scale, estimated_max_steering_deg=max_steer, minimum_radius_m=min_radius)
        rows.append(meta)
        spawn_lines += [f"  {name}:", f"    x: {meta['spawn_x']:.5f}", f"    y: {meta['spawn_y']:.5f}", f"    z: {meta['spawn_z']:.3f}", f"    yaw_rad: {meta['spawn_yaw_rad']:.7f}"]

    specs = [
        "project:", "  workspace: urrc_2", "  scale_class: 1/10",
        "vehicle_envelope_m:", f"  max_length: {VEHICLE_MAX_LENGTH}", f"  max_width: {VEHICLE_MAX_WIDTH}", f"  max_height: {VEHICLE_MAX_HEIGHT}",
        "track:", f"  width_m: {TRACK_WIDTH}", "  surface: uniform_very_dark_gray", "  floor_rgb: [0.045, 0.045, 0.050]", "  road_rgb: [0.055, 0.055, 0.060]", "  pit_lane: false",
        "cones:", "  color: yellow", f"  height_m: {CONE_HEIGHT}", f"  spacing_m: {CONE_SPACING}",
        "steering_validation:", f"  wheelbase_reference_m: {WHEELBASE_FOR_VALIDATION}", f"  max_allowed_deg: {MAX_ALLOWED_STEERING_DEG}", f"  design_target_deg: {DESIGN_STEERING_DEG}",
        "circuits: [monza, silverstone, spa, suzuka, monaco]", "development_default: monza",
    ]
    (config/"spawn_poses.yaml").write_text("\n".join(spawn_lines)+"\n", encoding="utf-8")
    (config/"track_specs.yaml").write_text("\n".join(specs)+"\n", encoding="utf-8")
    cols = ["circuit", "lap_length_m", "cones_total", "left_cone_spacing_m", "right_cone_spacing_m", "geometry_scale", "minimum_radius_m", "estimated_max_steering_deg", "max_elevation_m", "spawn_x", "spawn_y", "spawn_z", "spawn_yaw_rad"]
    with (config/"track_validation.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=cols)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row[key] for key in cols})
    write_preview(docs/"circuit_layouts.svg", prepared)
    print("Generated:")
    for row in rows:
        print(f"  {row['circuit']:11s} lap={row['lap_length_m']:.2f} m cones={int(row['cones_total']):3d} spacing={row['left_cone_spacing_m']:.3f}/{row['right_cone_spacing_m']:.3f} m minR={row['minimum_radius_m']:.2f} m steer={row['estimated_max_steering_deg']:.2f} deg")


if __name__ == "__main__":
    main()
