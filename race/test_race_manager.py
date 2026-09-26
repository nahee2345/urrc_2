import importlib.util
import json
from pathlib import Path
import zipfile
import pytest

SCRIPT=Path(__file__).parent/'scripts'/'race_manager.py'
spec=importlib.util.spec_from_file_location('race_manager',SCRIPT)
race=importlib.util.module_from_spec(spec);spec.loader.exec_module(race)

def test_lap_crossing_only_forward_and_inside_track():
    assert race.lap_crossing(-0.1,0.1,0.2,0.7)
    assert not race.lap_crossing(0.1,-0.1,0.2,0.7)
    assert not race.lap_crossing(-0.1,0.1,0.8,0.7)
    assert not race.lap_crossing(None,0.1,0.0,0.7)

def test_monza_event_persists(tmp_path):
    (tmp_path/'race').mkdir();(tmp_path/'race/results').mkdir()
    (tmp_path/'race/event_config.json').write_text(json.dumps({'track_pool':['monza']}))
    first=race.get_event(tmp_path);second=race.get_event(tmp_path)
    assert first==second and first['track'] == 'monza'
    third=race.get_event(tmp_path,new=True)
    assert third['event_id']!=first['event_id']

def test_reject_zip_path_escape(tmp_path):
    zpath=tmp_path/'bad.zip'
    with zipfile.ZipFile(zpath,'w') as z:z.writestr('../escape.txt','bad')
    with zipfile.ZipFile(zpath) as z:
        with pytest.raises(ValueError):race.safe_extract(z,tmp_path/'safe')

def test_entry_topics_must_be_unique_and_model_stay_inside_entry(tmp_path):
    entries=tmp_path/'race/entrants';entries.mkdir(parents=True)
    for i in range(10):
        folder=entries/f'team{i}';folder.mkdir();(folder/'car.sdf').write_text('<sdf/>')
        data={'entry_id':f'team{i}','display_name':f'Team {i}','ros_package':f'pkg{i}',
              'algorithm_launch_file':'race.launch.py','vehicle_model_file':'car.sdf',
              'speed_topic':'/same' if i<2 else f'/speed{i}',
              'steering_topic':f'/steer{i}','lidar_topic':f'/scan{i}'}
        (folder/'race_entry.json').write_text(json.dumps(data))
    with pytest.raises(ValueError,match='고유'):race.load_entries(tmp_path)

@pytest.mark.parametrize('count',[0,1,12])
def test_participant_count_accepts_zero_through_twelve(tmp_path,count):
    entries=tmp_path/'race/entrants';entries.mkdir(parents=True)
    for i in range(count):
        folder=entries/f'team{i}';folder.mkdir();(folder/'car.sdf').write_text('<sdf/>')
        manifest={'entry_id':f'team{i}','display_name':f'Team {i}','ros_package':f'pkg{i}',
                  'algorithm_launch_file':'race.launch.py','vehicle_model_file':'car.sdf',
                  'speed_topic':f'/speed{i}','steering_topic':f'/steer{i}','lidar_topic':f'/scan{i}'}
        (folder/'race_entry.json').write_text(json.dumps(manifest))
    assert len(race.load_entries(tmp_path))==count

def test_participant_count_rejects_more_than_twelve(tmp_path):
    entries=tmp_path/'race/entrants';entries.mkdir(parents=True)
    for i in range(13):
        folder=entries/f'team{i}';folder.mkdir();(folder/'car.sdf').write_text('<sdf/>')
        manifest={'entry_id':f'team{i}','display_name':f'Team {i}','ros_package':f'pkg{i}',
                  'algorithm_launch_file':'race.launch.py','vehicle_model_file':'car.sdf',
                  'speed_topic':f'/speed{i}','steering_topic':f'/steer{i}','lidar_topic':f'/scan{i}'}
        (folder/'race_entry.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError,match='최대 12개'):
        race.load_entries(tmp_path)

def test_all_worlds_have_off_f1_lights_and_tilted_gantry():
    import xml.etree.ElementTree as ET
    package=SCRIPT.parents[2]/'src'/'urrc_track_gazebo'
    for track in ('monza',):
        practice=ET.parse(package/'worlds'/f'{track}.sdf').getroot()
        assert practice.find(".//model[@name='race_start_gantry']") is None
        assert not any(l.get('name','').startswith('race_red_') for l in practice.findall('.//light'))
        root=ET.parse(package/'worlds'/f'{track}_race.sdf').getroot()
        world=root.find('world');gate=world.find("model[@name='race_start_gantry']")
        assert gate is not None
        assert len([l for l in world.findall('light') if l.get('name','').startswith('race_red_')])==5
        assert all(float(l.findtext('intensity'))==0 for l in world.findall('light') if l.get('name','').startswith('race_red_'))
        assert float(gate.find("link/visual[@name='red_lens_1']/pose").text.split()[4])==pytest.approx(-0.261799)
        pose=[float(v) for v in gate.findtext('pose').split()]
        assert (pose[0]**2+pose[1]**2)**.5==pytest.approx(1.648,abs=1e-5)
        lens_x=pose[0]-.148*__import__('math').cos(pose[5])
        lens_y=pose[1]-.148*__import__('math').sin(pose[5])
        assert (lens_x**2+lens_y**2)**.5==pytest.approx(1.5,abs=1e-5)
