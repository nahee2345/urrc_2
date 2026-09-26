# Gazebo Harmonic 8.11 collision import fix

The user's 2026-09-26 Monza runtime log showed successful world initialization
and GUI plugin loading, followed by:

```
[Err] [CustomMeshShape.cc:144] ... normal count [0] ... vertex count [186636]
Segmentation fault ... dart::collision::detail::OdeMesh::fillArrays
```

The old OBJ exporter wrote only `v` and `f` records. The DART/ODE mesh importer
requires normals. The new exporter writes indexed `vn` records and faces in
`f vertex//normal vertex//normal vertex//normal` format, and ships an MTL file
to avoid missing-material warnings. It also exports a dedicated planar-constrained
collision mesh with fewer triangles than the visual surface. No change to
centerline, wall height, grid positions or driving-surface footprint was made.

All 18 revised collision OBJs (three per circuit) were imported with Assimp
without normal generation flags; the imported normal count matched the imported
vertex count for every submesh. This reproduces and checks the specific import
contract that the earlier Gazebo 8.11 log reported as failing.

All six generated collision meshes must pass: equal vertex/normal record counts,
in-range normal indices, watertight solid, consistent face winding, positive
volume and reduced face count. Static validation cannot prove that Gazebo itself
will not crash. Test both server and GUI on the target machine:

```bash
cd ~/urrc_f1_ws
bash test_gazebo.sh
bash run_monza.sh
```

The GUI's Qt Quick `Binding loop detected` warnings in the supplied log are
separate and were not the cause identified in the native stack trace. If a
segmentation fault remains, keep the new `validation_results/gazebo/monza.log`
and the console output for further diagnosis.
