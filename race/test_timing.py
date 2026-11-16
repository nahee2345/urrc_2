import csv
import json
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'race/scripts'))
from timing import LapTimer, TimingError, crossing, duration, rank_rows, track_gates, world_base_sample


def square():
    return [dict(x=x,y=y,z=0,yaw=yaw,half_width=.8) for x,y,yaw in
            [(0,0,0),(2,1,math.pi/2),(0,2,math.pi),(-2,1,-math.pi/2)]]


def feed(timer,points,t=0,step=.01):
    for x,y in points:
        t+=step;timer.update((t,x,y,0))
    return t


def test_full_laps_exclude_grid_approach_and_freeze_finish():
    timer=LapTimer(square(),max_gap=.1,max_step=10)
    timer.update((0,-1,0,0));timer.arm(0,2)
    t=feed(timer,[(1,0)])
    circuit=[(2,0),(2,2),(-2,2),(-2,0),(1,0)]
    t=feed(timer,circuit,t)
    assert len(timer.lap_times)==1
    t=feed(timer,circuit,t)
    before=timer.snapshot()
    assert before['laps']==2
    assert sum(before['lap_times_seconds'])+before['approach_seconds']==pytest.approx(before['finish_time'])
    feed(timer,circuit,t)
    assert timer.snapshot()['finish_time']==before['finish_time']
    assert timer.snapshot()['laps']==2


def test_back_and_forth_at_finish_does_not_count_laps():
    timer=LapTimer(square());timer.update((0,-.1,0,0));timer.arm(0,2)
    feed(timer,[(.1,0),(-.1,0),(.1,0),(-.1,0),(.1,0)])
    assert timer.snapshot()['laps']==0
    assert timer.snapshot()['next_checkpoint']==1


def test_out_of_order_checkpoints_do_not_count():
    timer=LapTimer(square(),max_step=10);timer.update((0,-1,0,0));timer.arm(0,2)
    feed(timer,[(1,0),(-2,2),(-2,0),(1,0)])
    assert not timer.lap_times


def test_crossing_interpolates_time_and_lateral_position():
    gate=square()[0]
    hit=crossing((10,-1,0,0),(10.02,1,1,0),gate)
    assert hit['time']==pytest.approx(10.01)
    assert hit['bracket']==[10,10.02]
    assert hit['xyz']==pytest.approx([0,.5,0])
    assert crossing((10,1,0,0),(10.02,-1,0,0),gate) is None
    assert crossing((10,-1,0,5),(10.02,1,0,5),gate) is None


@pytest.mark.parametrize('sample',[(.2,.1,0,0),(.01,10,0,0),(-1,.1,0,0),(.01,math.nan,0,0)])
def test_missing_or_invalid_telemetry_rejected(sample):
    timer=LapTimer(square());timer.update((0,-.1,0,0));timer.arm(0,2)
    with pytest.raises(TimingError):timer.update(sample)


def tf(parent,child,xyz=(0,0,0),q=(0,0,0,1),t=1):
    return NS(header=NS(frame_id=parent,stamp=NS(sec=t,nanosec=0)),child_frame_id=child,
              transform=NS(translation=NS(x=xyz[0],y=xyz[1],z=xyz[2]),
                           rotation=NS(x=q[0],y=q[1],z=q[2],w=q[3])))


def test_actual_base_link_offset_is_rotated_into_world_and_teams_are_isolated():
    msg=[tf('monza','team01',(10,20,3),(0,0,math.sin(math.pi/4),math.cos(math.pi/4))),
         tf('team01','team01/base_link',(2,0,.5)),
         tf('monza','team02',(99,99,99)),tf('team02','team02/base_link',(5,0,0))]
    assert world_base_sample(msg,'team01','base_link','monza')==pytest.approx((1,10,22,3.5))


def test_frame_timestamp_mismatch_is_not_replaced_by_clock():
    msg=[tf('monza','team01'),tf('team01','team01/base_link',t=2)]
    with pytest.raises(TimingError):world_base_sample(msg,'team01','base_link','monza')
    with pytest.raises(TimingError):world_base_sample([tf('','team01')],'team01','base_link','monza')


@pytest.mark.parametrize('track',['monza','spa','monaco','cota','hungaroring','baku'])
def test_two_complete_laps_on_each_actual_centerline(track):
    gates=track_gates(ROOT,track)
    assert len(gates)>30
    timer=LapTimer(gates,max_gap=.05,max_step=3)
    first=gates[0]
    timer.update((0,first['x']-.2*math.cos(first['yaw']),first['y']-.2*math.sin(first['yaw']),first['z']))
    timer.arm(0,2)
    path=ROOT/f'src/urrc_track_gazebo/tracks/processed/{track}_centerline.csv'
    with path.open() as f:points=[tuple(float(row[k]) for k in ('x_m','y_m','z_m')) for row in csv.DictReader(f)]
    t=0.
    for point in [points[0],*points[1:],*points,*[points[0]]]:
        t+=.01;timer.update((t,*point))
    assert timer.snapshot()['laps']==2
    assert timer.snapshot()['finish_time']==pytest.approx(t)


def row(eid,status,laps,elapsed=None,last=None,bracket=None):
    return dict(entry_id=eid,status=status,laps=laps,time_seconds=elapsed,
                last_lap_elapsed_seconds=last,elapsed_bracket_seconds=bracket,grid_position=1)


def test_dnf_completed_laps_then_last_lap_elapsed():
    rows=[row('zero','DNF',0),row('slow','DNF',1,last=70),row('fast','DNF',1,last=60),
          row('done','finished',2,100,100,[99.99,100.01])]
    assert [r['entry_id'] for r in rank_rows(rows)]==['done','fast','slow','zero']


def test_overlapping_finish_brackets_require_review_not_false_precision():
    rows=[row('b','finished',5,100.006,100.006,[100,100.01]),
          row('a','finished',5,100.004,100.004,[100,100.01])]
    rank_rows(rows)
    assert [r['rank'] for r in rows]==[1,1]
    assert all(r['rank_provisional'] for r in rows)
    assert duration(125.1234)=='02분 05.123초'


def test_pause_and_arm_before_release_lights_out_and_resume(monkeypatch):
    import race_manager as race
    calls=[]
    monkeypatch.setattr(race,'pause_world',lambda track,paused,env:calls.append(('pause',paused)))
    monkeypatch.setattr(race,'lights_all',lambda track,root,on:calls.append(('lights',on)))
    monkeypatch.setattr(race,'light_request',lambda *args:None)
    monkeypatch.setattr(race.time,'sleep',lambda t:None)
    monitor=NS(frozen_time=lambda:123.,arm=lambda ids,laps,t:calls.append(('arm',t)),note_resume=lambda:None)
    gate=NS(publish=lambda ids,value:calls.append(('signal',value)))
    assert race.start_sequence('monza',ROOT,[{'entry_id':'a'}],.2,monitor,gate,2,{})==123
    assert calls==[('signal',False),('lights',False),('pause',True),('arm',123),
                   ('signal',True),('lights',False),('pause',False)]


def test_saved_pose_replay_detects_modified_lap_result(tmp_path):
    from replay_timing import replay
    import race_manager as race
    gates=square();timer=LapTimer(gates,max_step=10)
    timer.update((0,-1,0,0));timer.arm(0,2)
    log=[dict(type='gates',gates=gates),dict(type='arm',entry_id='a',previous=timer.previous,start_time=0,laps=2)]
    t=0
    for point in [(1,0)]+[(2,0),(2,2),(-2,2),(-2,0),(1,0)]*2:
        t+=.01;sample=(t,*point,0)
        log.append(dict(type='pose',entry_id='a',sample=sample));timer.update(sample)
    row=race.result_row(dict(entry_id='a',display_name='a',ros_package='a'),timer.snapshot())
    payload=dict(valid=True,results=[row],timing_config=dict(max_pose_gap_seconds=.05,max_pose_step_m=10))
    (tmp_path/'timing.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in log))
    path=tmp_path/'qualifying.json';path.write_text(json.dumps(payload))
    replay(tmp_path)
    payload['results'][0]['lap_times_seconds'][0]+=1
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError,match='불일치'):replay(tmp_path)
