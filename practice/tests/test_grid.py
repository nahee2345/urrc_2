import importlib.util
import json
from pathlib import Path
import unittest
import tempfile
import xml.etree.ElementTree as ET

path = Path(__file__).resolve().parents[1] / 'practice_mode.py'
spec = importlib.util.spec_from_file_location('practice_mode_grid', path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

class GridTest(unittest.TestCase):
    def test_actual_grids(self):
        if not mod.GRID_META.is_file():
            self.skipTest('Original Monza map not yet bootstrapped')
        meta = json.loads(mod.GRID_META.read_text())
        coords = {int(g['position']):g for g in meta['grid']}
        starts = mod.grid_start_positions()
        self.assertEqual(starts[0][1],2)
        self.assertEqual(starts[1][1],max(coords))
        self.assertEqual(starts[0][0], coords[2])
        self.assertEqual(starts[1][0], coords[max(coords)])

    def test_exact_grid_pose_without_lateral_offset(self):
        grid = {'x': -0.9991878919, 'y': -3.5058237134,
                'road_z': 1.1879452429, 'yaw': 1.4514102979,
                'distance_behind_start_m': 3.6}
        points = [(0., 0., 1.2), (0., 10., 1.2),
                  (10., 10., 1.2), (10., 0., 1.2), (5., -5., 1.2)]
        model = mod.car_model('test', '.1 .2 .3', grid, .7, -.4,
                              points, Path('/unused.so'))
        pose = [float(v) for v in model.findtext('pose').split()]
        self.assertAlmostEqual(pose[0], grid['x'], 7)
        self.assertAlmostEqual(pose[1], grid['y'], 7)
        self.assertAlmostEqual(pose[2], grid['road_z'] + .09, 7)
        self.assertAlmostEqual(pose[5], grid['yaw'], 7)
        plugin = model.find('plugin')
        self.assertAlmostEqual(float(plugin.findtext('spawn_x')), grid['x'])
        self.assertAlmostEqual(float(plugin.findtext('spawn_y')), grid['y'])

if __name__ == '__main__': unittest.main()
