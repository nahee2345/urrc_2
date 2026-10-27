"""Signal-only regression tests: copied v10.8 geometry and shared practice gate.

No Gazebo executable required; these tests DO NOT replace a Gazebo GUI test.
"""
import importlib.util
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"race"/"scripts"))
import atomic_signal as lights

class StartSignalTest(unittest.TestCase):
    def test_template_has_exactly_five_independent_visuals(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp)
            dest=root/"src/urrc_track_gazebo/worlds"
            dest.mkdir(parents=True)
            gate=ET.Element("model",name="race_start_gantry")
            ET.SubElement(gate,"pose").text="1 2 3 0 0 1"
            link=ET.SubElement(gate,"link",name="gantry_visuals")
            for n in range(1,6):
                visual=ET.SubElement(link,"visual",name=f"red_lens_{n}")
                ET.SubElement(visual,"pose").text=f"-0.148 {(n-3)*.16} 1.96 0 -.261799 0"
                ET.SubElement(ET.SubElement(ET.SubElement(visual,"geometry"),"box"),"size").text=".025 .11 .12"
                mat=ET.SubElement(visual,"material")
                ET.SubElement(mat,"diffuse").text=".12 .006 .008 1"
            sdf=ET.Element("sdf",version="1.9")
            world=ET.SubElement(sdf,"world",name="monza")
            world.append(gate)
            ET.ElementTree(sdf).write(dest/"monza_race.sdf")
            fn=lights._active_template(root,"monza")
            active=ET.parse(fn)
            model=active.find("model")
            self.assertEqual(model.attrib["name"],"urrc_start_signal_active")
            self.assertEqual(model.findtext("pose"),"1 2 3 0 0 1")
            visuals=active.findall(".//link[@name='signal_active_visuals']/visual")
            self.assertEqual([x.attrib["name"] for x in visuals],
                [f"active_red_lens_{i}" for i in range(1,6)])
            for n,v in enumerate(visuals,1):
                self.assertEqual(v.findtext("geometry/box/size"),".008 .110 .120")
                self.assertEqual(v.findtext("material/emissive"),"0 0 0 1")
                self.assertAlmostEqual(float(v.findtext("pose").split()[0]),-.168)
                self.assertAlmostEqual(float(v.findtext("pose").split()[1]),(n-3)*.16)
            self.assertEqual(len(ET.parse(dest/"monza_race.sdf").findall(".//visual")),5)

    def test_500_random_schedules_each_keep_strict_3_to_6_seconds(self):
        totals=[]
        for seed in range(500):
            gaps,hold=lights.schedule(seed)
            self.assertEqual(len(gaps),4)
            self.assertTrue(all(g>=.35 for g in gaps))
            self.assertTrue(.40<=hold<=.90)
            total=sum(gaps)+hold
            self.assertGreater(total,3)
            self.assertLess(total,6)
            totals.append(round(total,4))
        self.assertGreater(len(set(totals)),450)

    def test_five_individual_on_then_only_one_all_off(self):
        events=[]
        def on(track,index,enabled,root,env=None):
            self.assertTrue(enabled)
            events.append(("ON",index))
            return lights.time.monotonic()
        def off(track,root,env=None,missing_ok=True):
            events.append(("OFF",missing_ok))
            return True
        with patch.object(lights,"lamp",side_effect=on), \
             patch.object(lights,"all_off",side_effect=off), \
             patch.object(lights.time,"sleep",return_value=None):
            lights.sequence("monza",ROOT,seed=16)
        self.assertEqual([e for e in events if e[0]=="ON"],
                         [("ON",i) for i in range(5)])
        self.assertEqual(events[0][0],"OFF")
        self.assertEqual(events[-1],("OFF",False))
        self.assertEqual(sum(e[0]=="OFF" for e in events),2)  # reset + final

    def test_failed_off_never_emits_success(self):
        logs=[]
        def off(track,root,env=None,missing_ok=True):
            logs.append(missing_ok)
            if not missing_ok:
                raise RuntimeError("Gazebo removal failed")
            return True
        with patch.object(lights,"lamp",return_value=lights.time.monotonic()), \
             patch.object(lights,"all_off",side_effect=off), \
             patch.object(lights.time,"sleep",return_value=None):
            with self.assertRaises(RuntimeError):
                lights.sequence("monza",ROOT,seed=1)
        self.assertEqual(logs,[True,False,True])

    def test_partition_environment_forwarded_to_gazebo(self):
        env={"GZ_PARTITION":"different_partition"}
        result=types.SimpleNamespace(returncode=0,stdout="data: true",stderr="")
        with patch.object(lights.subprocess,"run",return_value=result) as call:
            lights._call("monza","visual_config","gz.msgs.Visual","name: \"test\"",
                        env=env)
        self.assertIs(call.call_args.kwargs["env"],env)

    def test_practice_releases_only_after_shared_sequence(self):
        file_path=ROOT/"practice"/"practice_mode.py"
        spec=importlib.util.spec_from_file_location("urrc_practice_under_test",file_path)
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        proc=types.SimpleNamespace(poll=lambda:None)
        env={"GZ_PARTITION":"practice_test"}
        with tempfile.TemporaryDirectory() as t:
            flag=Path(t)/"GO.flag"
            with patch.object(module,"_wait_visual_service"), \
                 patch.object(lights,"sequence",return_value=4.2) as seq:
                module._light_sequence(proc,env,flag)
                self.assertEqual(flag.read_text(),"GO\n")
                self.assertIs(seq.call_args.kwargs["env"],env)
            flag.unlink()
            with patch.object(module,"_wait_visual_service"), \
                 patch.object(lights,"sequence",side_effect=RuntimeError("light off failed")):
                with self.assertRaises(RuntimeError):
                    module._light_sequence(proc,env,flag)
                self.assertFalse(flag.exists())

if __name__=="__main__":
    unittest.main()
