"""Exercise orchestration with fake processes; these are not Gazebo drive tests."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'race/scripts'))
import race_manager as race


@pytest.mark.parametrize('mode,start_mode,broken_clock',[
    ('qualifying','signal',False),('qualifying','camera',False),
    ('final','signal',False),('final','camera',False),('final','signal',True)])
def test_entry_failure_does_not_stop_other_cars_and_results_persist(tmp_path,monkeypatch,mode,start_mode,broken_clock):
    (tmp_path/'race/results').mkdir(parents=True)
    (tmp_path/'race/event_config.json').write_text((ROOT/'race/event_config.json').read_text())
    event={'event_id':'test','track':'monza','seed':1}
    entries=[dict(entry_id=e,display_name=e,ros_package=e,algorithm_launch_file='race.launch.py',
                  speed_topic=f'/{e}/speed',steering_topic=f'/{e}/steer',lidar_topic=f'/{e}/scan') for e in ('bad','fast','slow')]
    monkeypatch.setattr(race,'validate',lambda root:entries)
    monkeypatch.setattr(race,'get_event',lambda root:event)
    monkeypatch.setattr(race,'entry_fingerprints',lambda entries:{'hash':'same'})
    monkeypatch.setattr(race,'grid',lambda *args:[{}]*12)
    if mode=='final':
        (tmp_path/'race/results/qualifying.json').write_text(json.dumps(dict(valid=True,provisional=False,event=event,track='monza',entry_fingerprints={'hash':'same'},results=entries)))
    removed=[];stopped=[];signals=[]
    class Proc:
        def poll(self):return None
    monkeypatch.setattr(race,'start_world',lambda *args:(Proc(),Proc(),{},NS(close=lambda:None)))
    monkeypatch.setattr(race,'spawn',lambda *args:None)
    monkeypatch.setattr(race,'remove',lambda track,eid,env:removed.append(eid))
    monkeypatch.setattr(race,'stop',lambda proc:stopped.append(proc))
    def launch(e,env,path):
        assert f'race_start_mode:={start_mode}' in race.launch_args(e)
        if e['entry_id']=='bad':raise OSError('broken team launch')
        return Proc()
    monkeypatch.setattr(race,'launch_algo',launch)
    monkeypatch.setattr(race,'pause_world',lambda *args:None)
    monkeypatch.setattr(race,'light_request',lambda *args:None)
    monkeypatch.setattr(race,'lights_all',lambda *args:None)
    monkeypatch.setattr(race.time,'sleep',lambda t:None)
    class Monitor:
        def __init__(self,*args):self.active={}
        def check(self):pass
        def wait_seen(self,eid):return True
        def frozen_time(self):return 100.
        def arm(self,ids,laps,start):self.active.update({eid:laps for eid in ids})
        def note_resume(self):pass
        def snapshot(self,eid):
            if eid not in self.active:return {'laps':0,'finish_time':None}
            laps=self.active[eid];elapsed=40 if eid=='fast' else 50
            return dict(laps=laps,finish_time=100.+elapsed,finish_bracket=[99.995+elapsed,100.005+elapsed],
                        start_time=100.,last_lap_time=100.+elapsed,first_crossing=101.,
                        approach_seconds=1.,lap_times_seconds=[(elapsed-1)/laps]*laps,max_sample_gap_seconds=.005)
        def deactivate(self,eid):pass
        def health(self,ids):
            if broken_clock:raise race.TimingError('clock stopped')
        def close(self):pass
    class Gate:
        def __init__(self,entries):self.pubs={e['entry_id']:NS(get_subscription_count=lambda:1) for e in entries}
        def ready(self,*args):return []
        def publish(self,ids,value):signals.append((ids,value))
        def close(self):pass
    monkeypatch.setattr(race,'PoseLapMonitor',Monitor)
    monkeypatch.setattr(race,'StartGate',Gate)
    if broken_clock:
        with pytest.raises(race.TimingError,match='clock stopped'):
            race.run_mode(tmp_path,mode,5 if mode=='final' else None,start_mode)
        payload=json.loads((tmp_path/f'race/results/{mode}.json').read_text())
        assert payload['valid'] is False and payload['state']=='invalid'
        assert payload['error']=='clock stopped'
        assert stopped
        return
    race.run_mode(tmp_path,mode,5 if mode=='final' else None,start_mode)
    payload=json.loads((tmp_path/f'race/results/{mode}.json').read_text())
    assert payload['valid'] and not payload['provisional']
    assert payload['laps']==(5 if mode=='final' else 2)
    assert [r['entry_id'] for r in payload['results']]==['fast','slow','bad']
    assert payload['results'][-1]['status']=='DNF'
    assert 'broken team launch' in payload['results'][-1]['reason']
    assert set(removed)=={'bad','fast','slow'}
    assert all('bad' not in ids for ids,value in signals if value)
    assert (tmp_path/f'race/results/{mode}_laps.csv').is_file()
    assert len(list((tmp_path/'race/results/runs').glob(f'*/{mode}.json')))==1


def test_final_rejects_changed_entry_after_qualifying(tmp_path,monkeypatch):
    (tmp_path/'race/results').mkdir(parents=True)
    (tmp_path/'race/event_config.json').write_text((ROOT/'race/event_config.json').read_text())
    event={'event_id':'e','track':'monza'}
    monkeypatch.setattr(race,'validate',lambda root:[{'entry_id':'team01'}])
    monkeypatch.setattr(race,'get_event',lambda root:event)
    monkeypatch.setattr(race,'entry_fingerprints',lambda entries:{'team01':'new'})
    (tmp_path/'race/results/qualifying.json').write_text(json.dumps(dict(valid=True,event=event,track='monza',entry_fingerprints={'team01':'old'})))
    with pytest.raises(ValueError,match='변경'):race.run_mode(tmp_path,'final',5,'signal')


@pytest.mark.parametrize('laps',[0,4,11])
def test_final_lap_range(tmp_path,laps):
    (tmp_path/'race').mkdir()
    (tmp_path/'race/event_config.json').write_text((ROOT/'race/event_config.json').read_text())
    with pytest.raises(ValueError,match='5~10'):race.run_mode(tmp_path,'final',laps)
