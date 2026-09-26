"""Exact metric inspection figures, not generated pictures of circuits."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection


def preview(path, track, road, slots):
    fig, ax = plt.subplots(figsize=(10, 10), facecolor="#f4f5f6")
    ax.set_facecolor("#f4f5f6")
    for ring in [road.exterior, *road.interiors]:
        xy = np.asarray(ring.coords)
        ax.plot(*xy.T, color="#282f35", linewidth=.65)
    raw = track.raw_xy - track.origin_xy
    ax.plot(*raw.T, color="#e67e22", linewidth=.8, alpha=.6, label="Source coordinates, scale 0.2")
    ax.plot(*np.vstack([track.xy, track.xy[0]]).T, color="#196d96", linewidth=.55, label="Generated centerline")
    ax.scatter([0], [0], color="#d7273f", s=30, label="Start / finish", zorder=4)
    ax.set_title(f"{track.name.upper()}   /   1:5\n{track.centerline.length:.2f} m    |    walls 0.80 m    |    grid 20", loc="left", fontsize=15)
    ax.set_aspect("equal"); ax.set_xlabel("East / m"); ax.set_ylabel("North / m")
    ax.legend(loc="best", fontsize=8); ax.grid(alpha=.13)
    fig.tight_layout(); fig.savefig(path / f"{track.name}_top.png", dpi=160); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    distance = np.r_[0, np.cumsum(np.linalg.norm(np.diff(track.xy, axis=0), axis=1))]
    axes[0].plot(distance, track.z, color="#196d96"); axes[0].set_ylabel("Road height / m")
    axes[1].plot(distance, track.widths, color="#3e7953"); axes[1].set_ylabel("Internal road width / m")
    axes[1].set_xlabel("Distance from start / m")
    axes[0].set_title("Elevation: smoothed terrain DEM calibrated to published range; widths: estimated profile", fontsize=10)
    for ax in axes: ax.grid(alpha=.15)
    fig.tight_layout(); fig.savefig(path / f"{track.name}_profile.png", dpi=130); plt.close(fig)


def overview(path, generated):
    fig, axes = plt.subplots(2, 3, figsize=(15, 10), facecolor="#f3f4f5")
    for ax, (track, road) in zip(axes.ravel(), generated):
        ax.set_facecolor("#f3f4f5")
        ax.fill(*np.asarray(road.exterior.coords).T, color="#31383e")
        for ring in road.interiors:
            ax.fill(*np.asarray(ring.coords).T, color="#f3f4f5")
        ax.scatter([0], [0], c="#dc3345", s=13, zorder=4)
        ax.set_aspect("equal"); ax.set_axis_off()
        ax.set_title(f"{track.name.upper()}\n{track.centerline.length:.1f} m", loc="left", fontsize=13)
    fig.suptitle("URRC / SIX CIRCUITS\n1:5 geometry · 0.80 m walls · 20 grid positions", fontsize=19, x=.05, ha="left")
    fig.text(.05, .01, "Top views use independent fit-to-panel zoom. Public coordinates; width/elevation approximations are documented.", fontsize=9)
    fig.tight_layout(rect=[0,.025,1,.90]); fig.savefig(path / "all_tracks.png", dpi=170); plt.close(fig)


def grid_preview(path, track):
    import json
    import trimesh
    package = path.parents[1]
    grid = json.loads((package / "tracks/processed" / f"{track.name}.json").read_text())["grid"]
    origin = np.array([grid[0]["x"], grid[0]["y"]])
    theta = grid[0]["yaw"]
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    fig, ax = plt.subplots(figsize=(14, 3.5), facecolor="#f3f4f5")
    ax.set_facecolor("#f3f4f5")
    for key, color in [("road", "#343b41"), ("wall_outer_red", "#c8323b"),
                       ("wall_outer_white", "#d5d8dc"), ("wall_inner_red", "#c8323b"),
                       ("wall_inner_white", "#d5d8dc"), ("markings", "#ffffff")]:
        mesh = trimesh.load(package / "models" / f"urrc_{track.name}" / "meshes" / f"{key}.obj", force="mesh", process=False)
        vertices = (mesh.vertices[:, :2] - origin) @ rotation
        triangles = vertices[mesh.faces]
        centers = triangles.mean(axis=1)
        keep = (centers[:, 0] > -35) & (centers[:, 0] < 6) & (np.abs(centers[:, 1]) < 5) & (mesh.face_normals[:, 2] > .5)
        ax.add_collection(PolyCollection(triangles[keep], facecolor=color, edgecolor="none", antialiased=False))
    for position in grid:
        x, y = (np.array([position["x"], position["y"]]) - origin) @ rotation
        ax.text(x, y, str(position["position"]), ha="center", va="center", color="#eab46b", fontsize=8)
    ax.set_xlim(-34, 5); ax.set_ylim(-2.5, 3.5); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(f"{track.name.upper()} / 20 GRID POSITIONS\nNumbers appear only in this preview. Gazebo road markings are white lines only.", loc="left", fontsize=13)
    fig.tight_layout(); fig.savefig(path / f"{track.name}_grid_detail.png", dpi=170); plt.close(fig)
