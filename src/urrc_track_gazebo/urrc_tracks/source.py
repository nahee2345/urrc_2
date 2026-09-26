"""Metric source loading. No lap-length normalization or anisotropic scaling."""
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml
from pyproj import CRS, Transformer
from scipy.interpolate import CubicHermiteSpline, CubicSpline, PchipInterpolator
from scipy.ndimage import gaussian_filter1d
from scipy.spatial import cKDTree
from shapely.geometry import LineString


class Track:
    def __init__(self, package: Path, name: str):
        self.name = name
        self.config = yaml.safe_load((package / "tracks/config" / f"{name}.yaml").read_text())
        c = self.config
        raw = package / "tracks/raw" / f"{name}.geojson"
        expected = json.loads((raw.parent / "SOURCES.json").read_text())[name]["sha256"]
        if hashlib.sha256(raw.read_bytes()).hexdigest() != expected:
            raise ValueError(f"{name}: raw source checksum mismatch")
        lonlat = np.asarray(json.loads(raw.read_text())["features"][0]["geometry"]["coordinates"], float)
        if not np.isfinite(lonlat).all() or not np.allclose(lonlat[0], lonlat[-1], atol=1e-9, rtol=0):
            raise ValueError("Source must be finite and explicitly closed")
        crs = CRS.from_proj4(f"+proj=aeqd +lat_0={lonlat[0, 1]} +lon_0={lonlat[0, 0]} +datum=WGS84 +units=m")
        transform = Transformer.from_crs(4326, crs, always_xy=True)
        xy = np.column_stack(transform.transform(lonlat[:, 0], lonlat[:, 1]))
        self.crs_wkt = crs.to_wkt()
        self.lonlat = lonlat
        self.scale = float(c["scale"])
        xy *= self.scale
        lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        if np.any(lengths < 1e-7):
            raise ValueError("Duplicate consecutive source vertices")
        self.raw_s = np.r_[0., np.cumsum(lengths)]
        self.period = float(self.raw_s[-1])
        self.raw_xy = xy
        unit = np.diff(xy, axis=0) / lengths[:, None]
        tangent = unit + np.roll(unit, 1, axis=0)
        tangent /= np.linalg.norm(tangent, axis=1)[:, None]
        self.curve = CubicHermiteSpline(self.raw_s, xy, np.vstack([tangent, tangent[0]]))
        self.origin_parameter = float(self.raw_s[c["start_source_vertex"]])
        self.origin_xy = self.curve(self.origin_parameter)
        # Profiles use original source vertex indices, so moving the start line
        # cannot silently relocate narrow corners or elevation landmarks.
        knots = np.asarray(c["width_profile_source_vertices"], float)
        k = np.interp(knots[:, 0], np.arange(len(self.raw_s)), self.raw_s)
        self.width_curve = PchipInterpolator(k, knots[:, 1] * self.scale)
        dem = json.loads((raw.parent / f"{name}_dem.json").read_text())
        values = np.array([r["elevation"] for r in dem["results"]], float)
        smoothed = gaussian_filter1d(values, c["elevation"]["dem_smoothing_sigma_samples"], mode="wrap")
        # SRTM is terrain, not road level. Only the broad profile is used;
        # its range is calibrated to a published F1 elevation-range reference.
        z = (smoothed - smoothed.min()) / np.ptp(smoothed)
        z *= c["elevation"]["reference_range_real_m"] * self.scale
        self.z_curve = CubicSpline(np.linspace(0, self.period, len(z) + 1), np.r_[z, z[0]], bc_type="periodic")
        self.z_offset = -float(self.z_curve(np.linspace(0, self.period, 20000)).min()) + .15
        self.samples = np.linspace(0, self.period, int(np.ceil(self.period / c["sampling_m"])), endpoint=False)
        self.xy = self.point(self.samples)
        self.z = self.height(self.samples)
        self.widths = self.width(self.samples)
        self.centerline = LineString(np.vstack([self.xy, self.xy[0]]))
        # Dense projection index used to lift planar mesh vertices onto the road.
        self.project_q = np.linspace(0, self.period, int(np.ceil(self.period / .10)), endpoint=False)
        self.project_xy = self.point(self.project_q)
        self.tree = cKDTree(self.project_xy)

    def raw_parameter(self, q):
        return (np.asarray(q) + self.origin_parameter) % self.period

    def point(self, q):
        return self.curve(self.raw_parameter(q)) - self.origin_xy

    def tangent(self, q):
        v = self.curve(self.raw_parameter(q), 1)
        return v / np.linalg.norm(v, axis=-1, keepdims=True)

    def normal(self, q):
        v = self.tangent(q)
        return np.stack([-v[..., 1], v[..., 0]], axis=-1)

    def width(self, q):
        return self.width_curve(self.raw_parameter(q))

    def height(self, q):
        return self.z_curve(self.raw_parameter(q)) + self.z_offset

    def project(self, points):
        """Nearest segment, including both neighbors and the closed seam."""
        points = np.atleast_2d(points)
        _, nearest = self.tree.query(points)
        n = len(self.project_xy)
        starts = np.column_stack([(nearest - 1) % n, nearest])
        a = self.project_xy[starts]
        b = self.project_xy[(starts + 1) % n]
        delta = b - a
        u = np.clip(np.sum((points[:, None, :] - a) * delta, axis=2) / np.sum(delta * delta, axis=2), 0, 1)
        closest = a + u[..., None] * delta
        which = np.argmin(np.sum((closest - points[:, None, :]) ** 2, axis=2), axis=1)
        row = np.arange(len(points))
        return (self.project_q[starts[row, which]] + u[row, which] * self.period / n) % self.period

    def surface_z(self, xy):
        return self.height(self.project(xy))
