"""Offline regression for exact urrc_f1_ws v10.8 start-signal logic.

Visual rendering and camera framerate must still be checked in Gazebo.
"""
import hashlib
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"race"/"scripts"))
import atomic_signal as bridge

SCRIPT=ROOT/"src/urrc_track_gazebo/scripts/traffic_lights_v10.py"
WORLD=ROOT/"src/urrc_track_gazebo/worlds/monza_race.sdf"
ACTIVE=ROOT/"src/urrc_track_gazebo/worlds/monza_signal_active.sdf"
F1_GITHUB_BLOB="8bfb8a44b74e42c3b71723aea38ba7aa0ce9d614"


def source():
    return bridge._module(ROOT)


class StartSignalTest(unittest.TestCase):
    def test_exact_f1_ws_source_code(self):
        contents=SCRIPT.read_bytes()
        digest=hashlib.sha1(b"blob "+str(len(contents)).encode()+b"\0"+contents).hexdigest()
        self.assertEqual(digest,F1_GITHUB_BLOB)

    def test_off_lenses_and_overlay_geometry(self):
        gate=ET.parse(WORLD).find(".//model[@name='race_start_gantry']")
        active=ET.parse(ACTIVE).find(".//model[@name='urrc_start_signal_active']")
        self.assertIsNotNone(gate)
        self.assertIsNotNone(active)
        self.assertEqual(gate.findtext("pose"),active.findtext("pose"))
        base=[gate.find(f"link[@name='gantry_visuals']/visual[@name='red_lens_{i}']") for i in range(1,6)]
        lit=[active.find(f"link[@name='signal_active_visuals']/visual[@name='active_red_lens_{i}']") for i in range(1,6)]
        self.assertTrue(all(x is not None for x in base+lit))
        for b,a in zip(base,lit):
            self.assertEqual(b.findtext("material/emissive"),"0 0 0 1")
            self.assertEqual(b.findtext("material/ambient"),"0.015 0.005 0.006 1")
            self.assertEqual(a.findtext("material/emissive"),"0 0 0 1")
            self.assertEqual(a.findtext("geometry/box/size"),".008 .110 .120")
            self.assertAlmostEqual(float(a.findtext("pose").split()[0]),float(b.findtext("pose").split()[0])-.020)
            self.assertAlmostEqual(float(a.findtext("pose").split()[1]),float(b.findtext("pose").split()[1]))
        self.assertEqual(len(active.findall(".//visual")),5)

    def test_500_random_schedules_3_to_6s(self):
        m=source()
        durations=set()
        for seed in range(500):
            plan=m.schedule(seed=seed)
            on=plan["on_s"]
            self.assertEqual(len(on),5)
            self.assertTrue(all(on[i]<on[i+1] for i in range(4)))
            self.assertLess(on[4],plan["off_s"])
            duration=plan["target_first_on_to_off_s"]
            self.assertGreater(duration,3)
            self.assertLess(duration,6)
            durations.add(round(duration,4))
        self.assertGreater(len(durations),450)

    def test_off_is_single_gazebo_entity_removal(self):
        m=source()
        logs=[]
        def fake_call(track,service,kind,payload,optional=False):
            logs.append((track,service,kind,payload,optional))
            return True
        with patch.object(m,"_call",side_effect=fake_call),patch.object(m.time,"monotonic",return_value=4.0):
            m._off("monza")
        self.assertEqual(len(logs),1)
        self.assertEqual(logs[0][1:3],("remove","gz.msgs.Entity"))
        self.assertIn('name: "urrc_start_signal_active" type: 2',logs[0][3])

    def test_five_ascending_on_then_one_off(self):
        m=source()
        events=[]
        with patch.object(m,"wait_ready",return_value=None), \
             patch.object(m.shutil,"which",return_value="/usr/bin/gz"), \
             patch.object(m,"_create",side_effect=lambda t:events.append(("CREATE",t))), \
             patch.object(m,"_red",side_effect=lambda t,i:(events.append(("ON",i)) or m.time.monotonic())), \
             patch.object(m,"_off",side_effect=lambda t:(events.append(("OFF",t)) or m.time.monotonic())), \
             patch.object(m.time,"sleep",return_value=None):
            m.run("monza",seed=42,check_cameras=False)
        self.assertEqual(events[0],("CREATE","monza"))
        self.assertEqual(events[1:6],[("ON",i) for i in range(1,6)])
        self.assertEqual(events[6:], [("OFF","monza")])

    def test_practice_and_race_use_canonical_script(self):
        with patch.object(bridge.subprocess,"run",return_value=types.SimpleNamespace(returncode=0)) as mock:
            bridge.sequence("monza",ROOT,env={"GZ_PARTITION":"testing"},seed=31)
        cmd=mock.call_args.args[0]
        self.assertEqual(Path(cmd[1]),SCRIPT)
        self.assertIn("--skip-camera-check",cmd)
        self.assertEqual(cmd[cmd.index("--seed")+1],"31")
        self.assertEqual(mock.call_args.kwargs["env"]["GZ_PARTITION"],"testing")
        with patch.object(bridge.subprocess,"run",return_value=types.SimpleNamespace(returncode=2)):
            with self.assertRaises(RuntimeError):
                bridge.sequence("monza",ROOT)

    def test_practice_release_only_after_success(self):
        file=ROOT/"practice/practice_mode.py"
        spec=importlib.util.spec_from_file_location("practice_for_signal_test",file)
        p=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(p)
        running=types.SimpleNamespace(poll=lambda:None)
        env={"GZ_PARTITION":"practice_test"}
        with tempfile.TemporaryDirectory() as t:
            gate=Path(t)/"start.flag"
            with patch.object(p,"_wait_visual_service"), \
                 patch.object(bridge,"sequence",return_value=0) as seq:
                p._light_sequence(running,env,gate)
            self.assertEqual(gate.read_text(),"GO\n")
            self.assertIs(seq.call_args.kwargs["env"],env)
            gate.unlink()
            with patch.object(p,"_wait_visual_service"), \
                 patch.object(bridge,"sequence",side_effect=RuntimeError("off service failed")):
                with self.assertRaises(RuntimeError):
                    p._light_sequence(running,env,gate)
            self.assertFalse(gate.exists())

    def test_manual_off_uses_same_source(self):
        m=source()
        with patch.object(m,"_call",return_value=True) as called:
            bridge.all_off("monza",ROOT)
        self.assertEqual(called.call_args.args[1],"remove")
        self.assertTrue(called.call_args.kwargs["optional"])

    def test_bare_world_skips_camera_gate_only(self):
        runner=(ROOT/"src/urrc_track_gazebo/scripts/run_world.py").read_text()
        self.assertIn('"--skip-camera-check"',runner)
        self.assertIn('traffic_lights_v10.py',runner)
        self.assertNotIn('range(4, -1, -1)',runner)


if __name__=="__main__":
    unittest.main()
