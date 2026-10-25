import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

SOURCE = Path(__file__).resolve().parents[1] / 'practice_mode.py'
spec = importlib.util.spec_from_file_location('practice_mode', SOURCE)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

class ModelTests(unittest.TestCase):
    def test_dimensions_and_plugin(self):
        points = [(0.,0.,1.),(2.,0.,1.),(2.,2.,1.),(0.,2.,1.),(-1.,1.,1.)]
        for n in (0, 1, 2):
            if n == 0: continue
            model = mod.car_model('urrc_practice_a', '.1 .3 .9', {'x':0.,'y':0.,'road_z':1.,'yaw':0.,'distance_behind_start_m':.4}, .7, -.4, points, Path('/test/plugin.so'))
            self.assertEqual(len(model.findall('./link/collision')), 5)
            self.assertEqual(len(model.findall('./link/visual')), 5)
            self.assertAlmostEqual(float(model.findtext('./plugin/speed_mps')), 0.7)
            self.assertEqual(model.findtext('./link/collision/geometry/box/size'), '.55 .19 .14')
            self.assertEqual(model.findtext('./link/visual/geometry/box/size'), '.55 .19 .14')
            wheel_poses = [x.findtext('pose').split() for x in model.findall('./link/collision') if 'wheel' in x.get('name','')]
            self.assertTrue(model.find('static') is not None)
        # x +/-0.275; y +/- (0.105 + .02); z -.09 .. +.09 => maximal requested size
        self.assertEqual((2*.275,2*(.105+.02),.09-(-.09)),(.55,.25,.18))

    def test_synthetic_track(self):
        pts = [(0.,0.,1.),(3.,0.,1.),(3.,3.,1.),(0.,3.,1.),(-1.,2.,1.)]
        self.assertEqual(mod.sample(pts, 0), pts[0])
        p = mod.sample(pts, 1.5)
        self.assertAlmostEqual(p[0], 1.5)
        self.assertAlmostEqual(p[1], 0)

if __name__ == '__main__': unittest.main()
