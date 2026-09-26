import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import threading
import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('race_manager',ROOT/'race/scripts/race_manager.py')
race=importlib.util.module_from_spec(spec);spec.loader.exec_module(race)


def make_package(base):
    p=base/'team01_race';(p/'models/team01_car').mkdir(parents=True)
    (p/'package.xml').write_text('<package><name>team01_race</name></package>')
    (p/'models/team01_car/model.sdf').write_text('<sdf version="1.9"><model name="team01"/></sdf>')
    data={'entry_id':'team01','display_name':'Team 01','ros_package':'team01_race',
          'algorithm_launch_file':'race.launch.py','vehicle_model_file':'src/team01_race/models/team01_car/model.sdf',
          'speed_topic':'/team01/speed','steering_topic':'/team01/steering','lidar_topic':'/team01/scan'}
    (p/'race_entry.json').write_text(json.dumps(data))
    return p


@pytest.mark.parametrize('style',['package','src','team','wrapped'])
def test_submission_layouts_are_portable(tmp_path,style):
    src=tmp_path/'a folder with spaces'/'src';package=make_package(src)
    source={'package':package,'src':src,'team':src.parent,'wrapped':tmp_path}[style]
    out=tmp_path.parent/(tmp_path.name+'-out')
    eid=race.normalize(source,out)
    assert eid=='team01'
    assert (out/'src/team01_race/models/team01_car/model.sdf').is_file()
    assert json.loads((out/'race_entry.json').read_text())['entry_id']=='team01'


def test_spawning_passes_gazebo_yaw_flag(tmp_path):
    entry={'entry_id':'team01','_model':str(tmp_path/'car.urdf')}
    with patch.object(race.subprocess,'run') as run:
        race.spawn(tmp_path,'spa',entry,{'road_z':2,'x':1,'y':3,'yaw':1.25},{'GZ_PARTITION':'test'})
    args=run.call_args.args[0]
    assert '-Y' in args and '--yaw' not in args
    assert args[args.index('-Y')+1]=='1.25'
    assert run.call_args.kwargs['env']['GZ_PARTITION']=='test'


def test_lens_control_rejects_boolean_failure():
    with patch.object(race.subprocess,'run',return_value=SimpleNamespace(stdout='data: false',stderr='')):
        with pytest.raises(RuntimeError):race.light_request('monza',0,True,ROOT)


def test_first_crossing_is_not_a_lap():
    monitor=race.PoseLapMonitor.__new__(race.PoseLapMonitor)
    monitor.cv=threading.Condition();monitor.x0=monitor.y0=monitor.yaw=0
    monitor.half_width=1;monitor.sim_time=0
    monitor.states={'team01':{'seen':True,'previous_s':-1,'laps':0,'armed':False}}
    monitor.arm(['team01'],2)
    def send(x,t):
        tf=SimpleNamespace(child_frame_id='team01',transform=SimpleNamespace(translation=SimpleNamespace(x=x,y=0)),
            header=SimpleNamespace(stamp=SimpleNamespace(sec=t,nanosec=0)))
        monitor.pose_cb(SimpleNamespace(transforms=[tf]))
    send(-1,1);send(1,2)
    assert monitor.states['team01']['laps']==0
    send(-1,3);send(1,4)
    assert monitor.states['team01']['laps']==1
    send(-1,5);send(1,6)
    assert monitor.states['team01']['laps']==2


def test_explicit_track_persists_for_final(tmp_path):
    (tmp_path/'race').mkdir()
    (tmp_path/'race/event_config.json').write_text(json.dumps({'track_pool':['monza','spa']}))
    event=race.get_event(tmp_path,new=True,track='spa')
    assert race.get_event(tmp_path)==event
    with pytest.raises(ValueError):race.get_event(tmp_path,new=True,track='unknown')
