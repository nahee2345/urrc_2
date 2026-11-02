import math
import sys
import unittest
from pathlib import Path

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from routes import TRACKS,GRID_SPEEDS,centerline,lane_route,projection
from run import confirmed_all_off
import json

ROOT=HERE.parents[2]
PKG=ROOT/'src'/'urrc_track_gazebo'

class TestRoutes(unittest.TestCase):
    def test_all_off_gate(self):
        for msg in ('RED 1/5 at 0.201s', 'RED 5/5 at 3.890s',
                    'LIGHTS OUT first_on_to_off=4.300s',
                    '[WARN] Gazebo service latency exceeded 3..6s target'):
            self.assertFalse(confirmed_all_off(msg))
        self.assertTrue(confirmed_all_off('LIGHTS OUT first_on_to_off=4.300s model_remove=single_request'))

    def test_grid_9_not_grid_10(self):
        self.assertEqual(GRID_SPEEDS, ((2, 10.0), (9, 20.0)))

    def test_grid_offsets_and_continuity(self):
        for track in TRACKS:
            data=json.loads((PKG/'tracks/processed'/f'{track}.json').read_text())
            samples=centerline(PKG/'tracks/processed'/f'{track}_centerline.csv')
            for slot,speed in GRID_SPEEDS:
                with self.subTest(track=track,slot=slot):
                    grid=data['grid'][slot-1]
                    route=lane_route(samples,grid)
                    first=route['samples'][0]
                    self.assertAlmostEqual(first[1],grid['x'],places=6)
                    self.assertAlmostEqual(first[2],grid['y'],places=6)
                    self.assertAlmostEqual(first[3],grid['road_z'],places=6)
                    self.assertEqual(route['samples'][-1][1:4],first[1:4])
                    self.assertGreater(route['length_m'],500)
                    self.assertLess(route['length_m'],1500)
                    # Every sub-sample must be continuous, never auto-center/teleport.
                    max_step=max(math.dist(a[1:4],b[1:4]) for a,b in zip(route['samples'],route['samples'][1:]))
                    self.assertLess(max_step,1.5)
                    self.assertGreater(abs(route['offset_m']),0.2)

if __name__=='__main__':unittest.main()
