"""Portable OBJ and SDFormat 1.9 export, with no home paths or online assets."""
from pathlib import Path
import io
import os
import xml.etree.ElementTree as ET
import numpy as np
import trimesh


def write_obj(path, vertices, faces):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    # Remove unreferenced vertices, especially from two wall color subsets.
    used, inverse = np.unique(np.asarray(faces).ravel(), return_inverse=True)
    vertices = np.asarray(vertices)[used]
    faces = inverse.reshape(-1, 3)
    # Gazebo Harmonic's DART + ODE mesh importer requires one valid normal for
    # each imported vertex. Plain "f v1 v2 v3" faces crashed in Gazebo 8.11.
    normals = trimesh.Trimesh(vertices=vertices, faces=faces, process=False).vertex_normals
    if not np.isfinite(normals).all() or len(normals) != len(vertices):
        raise ValueError(f"Missing or invalid OBJ normals: {path}")
    buffer = io.StringIO()
    buffer.write("# URRC; metres; Z up; static concave triangle mesh\n")
    buffer.write("mtllib urrc_default.mtl\no URRC\nusemtl URRCDefault\n")
    np.savetxt(buffer, vertices, fmt="v %.8f %.8f %.8f")
    np.savetxt(buffer, normals, fmt="vn %.8f %.8f %.8f")
    # Give each corner the corresponding vertex normal in OBJ's v//vn form.
    for a, b, c in faces + 1:
        buffer.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")
    data = buffer.getvalue().encode("ascii")
    temporary = path.with_suffix(".obj.tmp")
    with temporary.open("wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    if path.stat().st_size != len(data):
        raise IOError(f"Incomplete mesh write: {path}")
    material = path.parent / "urrc_default.mtl"
    if not material.exists():
        material.write_text("newmtl URRCDefault\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nd 1\n")


def child(parent, tag, text=None, **attrib):
    node = ET.SubElement(parent, tag, attrib)
    if text is not None:
        node.text = str(text)
    return node


def write_xml(root, path):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def material(visual, color):
    mat = child(visual, "material")
    child(mat, "ambient", color)
    child(mat, "diffuse", color)
    child(mat, "specular", "0.02 0.02 0.02 1")


def write_model(path, name):
    model_name = "urrc_" + name
    root = ET.Element("sdf", version="1.9")
    model = child(root, "model", name=model_name)
    child(model, "static", "true")
    link = child(model, "link", name="track")
    for mesh in ["road", "wall_outer", "wall_inner"]:
        collision = child(link, "collision", name=mesh + "_collision")
        geometry = child(collision, "geometry")
        child(child(geometry, "mesh"), "uri", f"model://{model_name}/meshes/{mesh}_collision.obj")
        surface = child(collision, "surface")
        friction = child(surface, "friction")
        ode = child(friction, "ode")
        child(ode, "mu", "1.0"); child(ode, "mu2", "1.0")
    colors = {"road": "0.14 0.15 0.16 1", "markings": "0.96 0.96 0.94 1",
              "wall_outer_red": "0.78 0.035 0.045 1", "wall_inner_red": "0.78 0.035 0.045 1",
              "wall_outer_white": "0.9 0.9 0.88 1", "wall_inner_white": "0.9 0.9 0.88 1"}
    for mesh, color in colors.items():
        visual = child(link, "visual", name=mesh + "_visual")
        child(visual, "cast_shadows", "false" if mesh == "markings" else "true")
        child(child(child(visual, "geometry"), "mesh"), "uri", f"model://{model_name}/meshes/{mesh}.obj")
        material(visual, color)
    write_xml(root, path / "model.sdf")
    config = ET.Element("model")
    child(config, "name", model_name); child(config, "version", "1.0")
    child(config, "sdf", "model.sdf", version="1.9")
    child(config, "description", "1/5 circuit with 0.8m boundary walls and 20 painted grid positions.")
    write_xml(config, path / "model.config")


def write_world(path, track, *, traffic_lights=False):
    root = ET.Element("sdf", version="1.9")
    world = child(root, "world", name=track.name)
    child(world, "gravity", "0 0 -9.81")
    physics = child(world, "physics", name="physics", type="ignored")
    child(physics, "max_step_size", ".001")
    child(physics, "real_time_factor", "1.0")
    for filename, name in [("physics", "Physics"), ("user-commands", "UserCommands"),
                           ("scene-broadcaster", "SceneBroadcaster"), ("sensors", "Sensors")]:
        plugin = child(world, "plugin", filename=f"gz-sim-{filename}-system", name=f"gz::sim::systems::{name}")
        if name == "Sensors":
            child(plugin, "render_engine", "ogre2")
    scene = child(world, "scene")
    child(scene, "ambient", "0.65 0.65 0.65 1")
    child(scene, "background", "0 0 0 1")
    child(scene, "grid", "false")
    child(scene, "origin_visual", "false")
    child(scene, "shadows", "true")
    light = child(world, "light", name="sun", type="directional")
    child(light, "pose", "0 0 100 0 0 0")
    child(light, "visualize", "false")
    child(light, "diffuse", "0.85 0.85 0.85 1")
    child(light, "specular", "0.1 0.1 0.1 1")
    child(light, "direction", "-0.3 0.2 -1")
    child(light, "cast_shadows", "true")
    attenuation = child(light, "attenuation")
    child(attenuation, "range", "2000"); child(attenuation, "constant", "1")
    child(attenuation, "linear", "0"); child(attenuation, "quadratic", "0")
    if traffic_lights:
        # F1-style start gantry: 1.5 m beyond the start line, lamps 2 m above road.
        tangent = track.tangent(0.)
        yaw = float(np.arctan2(tangent[1], tangent[0]))
        # The lens faces sit 0.148 m behind the gantry origin, so place the
        # gantry at 1.648 m to put the signal lenses exactly 1.5 m past start.
        gate_xy = track.point(0.) + tangent * 1.648
        road_z = float(track.height(0.))
        gate = child(world, "model", name="race_start_gantry")
        child(gate, "static", "true")
        child(gate, "pose", f"{gate_xy[0]:.8f} {gate_xy[1]:.8f} {road_z:.8f} 0 0 {yaw:.8f}")
        link = child(gate, "link", name="gantry_visuals")
        def box_visual(name, pose, size, color):
            visual = child(link, "visual", name=name)
            child(visual, "pose", pose)
            child(child(visual, "geometry"), "box")
            child(visual.find("geometry/box"), "size", size)
            material(visual, color)
            visual.find("material/specular").text = "0.02 0.02 0.02 1"
        half_span = float(track.width(0.)) / 2 + .20
        # Support uprights stay just outside the track walls; the gantry has no collision.
        for index, side in enumerate((-1, 1)):
            box_visual(f"support_{index}", f"0 {side*half_span:.5f} 1.03 0 0 0", ".10 .10 2.06", "0.08 0.08 0.09 1")
        box_visual("crossbar", "0 0 2.08 0 0 0", f".12 {2*half_span:.5f} .12", "0.06 0.06 0.07 1")
        box_visual("lamp_housing", "-0.09 0 1.96 0 -0.261799 0", ".10 .88 .22", "0.015 0.015 0.018 1")
        for index in range(5):
            lateral = (index - 2) * .16
            box_visual(f"red_lens_{index+1}", f"-0.148 {lateral:.4f} 1.96 0 -0.261799 0", ".025 .11 .12", "0.12 0.006 0.008 1")
            signal = child(world, "light", name=f"race_red_{index+1}", type="point")
            child(signal, "pose", f"{gate_xy[0] - .148*tangent[0] - lateral*np.sin(yaw):.8f} {gate_xy[1] - .148*tangent[1] + lateral*np.cos(yaw):.8f} {road_z+1.96:.8f} 0 0 0")
            child(signal, "diffuse", "1 0.01 0.01 1")
            child(signal, "specular", "1 0.02 0.02 1")
            child(signal, "intensity", "0")
            child(signal, "cast_shadows", "false")
            attenuation = child(signal, "attenuation")
            child(attenuation, "range", "8"); child(attenuation, "constant", "1")
            child(attenuation, "linear", ".15"); child(attenuation, "quadratic", ".01")
    # A single untextured safety ground lies below the lowest road point.
    floor = child(world, "model", name="ground")
    child(floor, "static", "true"); child(floor, "pose", "0 0 -0.05 0 0 0")
    link = child(floor, "link", name="ground")
    for tag in ["collision", "visual"]:
        elem = child(link, tag, name="ground_" + tag)
        plane = child(child(elem, "geometry"), "plane")
        child(plane, "normal", "0 0 1"); child(plane, "size", "2000 2000")
        if tag == "visual":
            material(elem, "0 0 0 1")
            elem.find("material/specular").text = "0 0 0 1"
    include = child(world, "include")
    child(include, "uri", "model://urrc_" + track.name)
    child(include, "name", "urrc_" + track.name)
    # GUI camera pose is supplied via an explicit, version-neutral GUI config.
    write_xml(root, path)


def write_gui(path, track, overview=True):
    # Standard Gazebo GUI plugins exist in both Harmonic and Jetty.
    root = ET.Element("gui")
    window = child(root, "window")
    child(window, "width", "1280"); child(window, "height", "900")
    child(window, "default_exit_action", "shutdown_server")
    child(window, "dialog_on_exit", "false")
    child(window, "server_control_service", f"/server_control")
    scene = child(root, "plugin", filename="MinimalScene", name="3D View")
    settings = child(scene, "gz-gui")
    child(settings, "title", "3D View")
    child(settings, "property", "false", type="bool", key="showTitleBar")
    child(settings, "property", "docked", type="string", key="state")
    child(scene, "engine", "ogre2")
    child(scene, "scene", "scene")
    child(scene, "ambient_light", "0.65 0.65 0.65")
    child(scene, "background_color", "0 0 0")
    lo, hi = track.xy.min(axis=0), track.xy.max(axis=0)
    center = (lo + hi) / 2
    if overview:
        altitude = max(float(hi[0]-lo[0]), float(hi[1]-lo[1])) * 1.5 + track.z.max()
        pose = f"{center[0]:.5f} {center[1]:.5f} {altitude:.5f} 0 1.57079632679 -1.57079632679"
    else:
        q = 0.; tangent = track.tangent(q); center = track.point(q) - tangent * 12
        pose = f"{center[0]:.5f} {center[1]:.5f} {float(track.height(q))+10:.5f} 0 .65 {np.arctan2(tangent[1],tangent[0]):.8f}"
    child(scene, "camera_pose", pose)
    child(scene, "camera_clip", None)
    clip = scene.find("camera_clip"); child(clip, "near", ".05"); child(clip, "far", "5000")
    for filename, name in [("GzSceneManager", "Scene Manager"), ("InteractiveViewControl", "Interactive view control"),
                           ("CameraTracking", "Camera tracking"), ("WorldControl", "World control"),
                           ("WorldStats", "World statistics"), ("EntityTree", "Entity tree")]:
        plugin = child(root, "plugin", filename=filename, name=name)
        settings = child(plugin, "gz-gui")
        child(settings, "property", "false", type="bool", key="showTitleBar")
        child(settings, "property", "docked" if filename == "EntityTree" else "floating", type="string", key="state")
        if filename not in ("WorldControl", "WorldStats", "EntityTree"):
            child(settings, "property", "5", type="double", key="width")
            child(settings, "property", "5", type="double", key="height")
        if filename in ("WorldControl", "WorldStats"):
            child(settings, "property", "false", type="bool", key="resizable")
            child(settings, "property", "72" if filename == "WorldControl" else "110", type="double", key="height")
            child(settings, "property", "290", type="double", key="width")
            child(settings, "property", "1", type="double", key="z")
            anchors = child(settings, "anchors", target="3D View")
            edge = "left" if filename == "WorldControl" else "right"
            child(anchors, "line", own=edge, target=edge); child(anchors, "line", own="bottom", target="bottom")
        if filename == "WorldControl":
            child(plugin, "play_pause", "true"); child(plugin, "step", "true"); child(plugin, "start_paused", "false")
            child(plugin, "use_event", "true")
        if filename == "WorldStats":
            for key in ("sim_time", "real_time", "real_time_factor", "iterations"):
                child(plugin, key, "true")
    # gz-gui Application reads top-level <window> and <plugin> siblings.
    # A <gui> wrapper is valid inside SDF, but NOT for --gui-config files.
    ET.indent(root, space="  ")
    path.write_text('<?xml version="1.0"?>\n' + '\n'.join(ET.tostring(elem, encoding="unicode") for elem in root) + '\n')
