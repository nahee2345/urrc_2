"""Single-hole road polygons and constrained, non-convex collision surfaces."""
import numpy as np
import triangle
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient


def make_road(track):
    normals = track.normal(track.samples)
    left = track.xy + normals * track.widths[:, None] / 2
    right = track.xy - normals * track.widths[:, None] / 2
    a, b = Polygon(left), Polygon(right)
    # Offset curves can fold locally inside a tight hairpin. Resolve their
    # planar union before triangulation, never export overlapping strip faces.
    cleaned_a, cleaned_b = a.buffer(0), b.buffer(0)
    if cleaned_a.geom_type != "Polygon" or cleaned_b.geom_type != "Polygon":
        raise ValueError("Offset topology split; source/profile needs correction")
    outer, inner = sorted([cleaned_a, cleaned_b], key=lambda x: x.area, reverse=True)
    if not outer.contains(inner):
        raise ValueError("Road boundaries cross; refusing to join road branches")
    road = orient(outer.difference(inner), sign=1.)
    if road.geom_type != "Polygon" or not road.is_valid or len(road.interiors) != 1:
        raise ValueError("Road must have exactly one closed driving corridor")
    return road, {"left_raw_offset_simple": bool(a.exterior.is_simple),
                  "right_raw_offset_simple": bool(b.exterior.is_simple),
                  "offset_trim_area_m2": float(a.symmetric_difference(cleaned_a).area) if a.is_valid else None}


def triangulate_surface(poly, max_area=.18):
    poly = orient(poly, sign=1.)
    rings = [np.asarray(poly.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in poly.interiors]
    vertices, segments, offset = [], [], 0
    for ring in rings:
        n = len(ring)
        vertices.extend(ring)
        segments.extend((offset + i, offset + (i + 1) % n) for i in range(n))
        offset += n
    data = {"vertices": np.asarray(vertices), "segments": np.asarray(segments)}
    if poly.interiors:
        data["holes"] = np.array([Polygon(r).representative_point().coords[0] for r in poly.interiors])
    options = "pQ" if max_area is None else f"pq20a{max_area}Q"
    result = triangle.triangulate(data, options)
    xy, faces = result["vertices"], result["triangles"].astype(int)
    delta1, delta2 = xy[faces[:, 1]] - xy[faces[:, 0]], xy[faces[:, 2]] - xy[faces[:, 0]]
    negative = delta1[:, 0] * delta2[:, 1] - delta1[:, 1] * delta2[:, 0] < 0
    faces[negative] = faces[negative][:, [0, 2, 1]]
    return xy, faces, result["segments"].astype(int)


def solid_road(poly, track, *, top_offset=0., depth=None, collision=False):
    xy, top, edges = triangulate_surface(poly, None if collision else track.config["mesh_max_triangle_area_m2"])
    v = np.column_stack([xy, track.surface_z(xy) + top_offset])
    n = len(v)
    if depth is None:
        depth = track.config["road_thickness_m"]
    vertices = np.vstack([v, v - [0, 0, depth]])
    # Directed boundary edges are recovered from the consistently CCW top mesh.
    all_edges = np.vstack([top[:, [0, 1]], top[:, [1, 2]], top[:, [2, 0]]])
    _, inverse, counts = np.unique(np.sort(all_edges, axis=1), axis=0, return_inverse=True, return_counts=True)
    boundary = all_edges[counts[inverse] == 1]
    sides = []
    for a, b in boundary:
        sides.extend([[b, a, a + n], [b, a + n, b + n]])
    faces = np.vstack([top, top[:, [0, 2, 1]] + n, sides])
    return vertices, faces, top, n


def make_wall_polygon(ring, thickness, outer):
    filled = Polygon(ring)
    # Exact planar offsets: trimming a concave wall corner must not reduce
    # road clearance or leave overlapping prism segments inside the lane.
    wall = filled.buffer(thickness, quad_segs=3).difference(filled) if outer else filled.difference(filled.buffer(-thickness, quad_segs=3))
    if wall.geom_type != "Polygon" or not wall.is_valid or len(wall.interiors) != 1:
        raise ValueError("Wall footprint pinches off; cannot retain a closed corridor")
    return orient(wall, sign=1.)


def paint_rectangle(track, center_q, offset, forward_start, forward_end, lateral_start, lateral_end):
    """Markings follow local road height and contain no collision geometry."""
    tangent, normal = track.tangent(center_q), track.normal(center_q)
    center = track.point(center_q) + normal * offset
    xy = np.array([center + tangent * f + normal * l for f, l in
                  [(forward_start, lateral_start), (forward_end, lateral_start),
                   (forward_end, lateral_end), (forward_start, lateral_end)]])
    return np.column_stack([xy, track.surface_z(xy) + .003]), np.array([[0, 1, 2], [0, 2, 3]])


def grid_markings(track):
    c = track.config["grid"]
    # Convert true arc length behind start to the interpolation parameter.
    q = np.linspace(0, track.period, 30000)
    xy = track.point(q)
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    total = s[-1]
    vertices, faces, slots = [], [], []
    half = float(track.width(0)) / 2
    v, f = paint_rectangle(track, 0, 0, -.025, .025, -half, half)
    vertices.extend(v); faces.extend(f)
    for i in range(c["count"]):
        distance = c["first_slot_back_m"] + i * c["successive_slot_spacing_m"]
        parameter = np.interp(total - distance, s, q)
        side = c["pole_side"] if i % 2 == 0 else -c["pole_side"]
        offset = side * float(track.width(parameter)) / 4
        length, width, thickness = c["box_length_m"], c["box_width_m"], c["line_width_m"]
        # Three white lines: transverse front line and two side lines.
        for f0, f1, l0, l1 in [(length/2-thickness, length/2, -width/2, width/2),
                               (-length/2, length/2, -width/2, -width/2+thickness),
                               (-length/2, length/2, width/2-thickness, width/2)]:
            v, f = paint_rectangle(track, parameter, offset, f0, f1, l0, l1)
            faces.extend(f + len(vertices)); vertices.extend(v)
        pos = track.point(parameter) + track.normal(parameter)*offset
        tangent = track.tangent(parameter)
        slots.append({"position": i+1, "x": float(pos[0]), "y": float(pos[1]),
                      "road_z": float(track.surface_z(pos)[0]), "yaw": float(np.arctan2(tangent[1], tangent[0])),
                      "distance_behind_start_m": distance, "source": "approximate F1-style painted grid"})
    return np.asarray(vertices), np.asarray(faces), slots
