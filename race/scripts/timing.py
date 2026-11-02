"""ROS-independent official timing: ordered gates and timestamped base_link poses."""
from __future__ import annotations
import csv
import json
import math


class TimingError(RuntimeError):
    """A trustworthy result cannot be produced from the telemetry."""


def crossing(a, b, gate):
    """Return interpolated time and its sample bracket, or None.

    Samples are (simulation_seconds, x, y, z); gates use a forward normal.
    Width and height are checked at the crossing, not at the next sample.
    """
    t0, x0, y0, z0 = a
    t1, x1, y1, z1 = b
    c, s = math.cos(gate['yaw']), math.sin(gate['yaw'])
    d0 = (x0-gate['x'])*c + (y0-gate['y'])*s
    d1 = (x1-gate['x'])*c + (y1-gate['y'])*s
    if not d0 < 0 <= d1 or t1 <= t0:
        return None
    f = -d0/(d1-d0)
    x, y, z = x0+f*(x1-x0), y0+f*(y1-y0), z0+f*(z1-z0)
    lateral = -(x-gate['x'])*s + (y-gate['y'])*c
    if abs(lateral) > gate['half_width'] or abs(z-gate['z']) > gate.get('height_tolerance', .6):
        return None
    return {'time': t0+f*(t1-t0), 'bracket': [t0, t1], 'xyz': [x,y,z]}


def track_gates(root, track, spacing=10.):
    folder = root/'src/urrc_track_gazebo/tracks/processed'
    meta = json.loads((folder/f'{track}.json').read_text())
    with (folder/f'{track}_centerline.csv').open() as f:
        rows = [{k:float(v) for k,v in row.items()} for row in csv.DictReader(f)]
    def gate(row):
        return dict(x=row['x_m'], y=row['y_m'], z=row['z_m'], yaw=row['yaw_rad'],
                    half_width=row['road_width_m']/2+.15, height_tolerance=.6)
    finish = dict(meta['start_finish'], half_width=rows[0]['road_width_m']/2+.15,
                  height_tolerance=.6)
    distance, next_at, checkpoints = 0., spacing, []
    for prev, row in zip(rows, rows[1:]):
        distance += math.dist([prev[k] for k in ('x_m','y_m','z_m')],
                              [row[k] for k in ('x_m','y_m','z_m')])
        if distance >= next_at:
            checkpoints.append(gate(row)); next_at += spacing
    # Keep the last checkpoint away from the finish gate.
    if checkpoints and distance-(next_at-spacing) < spacing*.5:
        checkpoints.pop()
    if len(checkpoints) < 3:
        raise ValueError('계측 체크포인트가 부족합니다.')
    return [finish, *checkpoints]


class LapTimer:
    def __init__(self, gates, max_gap=.05, max_step=3.):
        self.gates = gates
        self.max_gap, self.max_step = max_gap, max_step
        self.previous = None
        self.start_time = None
        self.target_laps = 0
        self.first_crossing = None
        self.finish_time = None
        self.finish_bracket = None
        self.next_gate = 0
        self.lap_times = []
        self.crossings = []
        self.last_lap_time = None
        self.active = False
        self.max_observed_gap = 0.

    def arm(self, start_time, laps):
        if self.previous is None:
            raise TimingError('base_link 포즈가 없습니다.')
        if not 0 <= start_time-self.previous[0] <= self.max_gap:
            raise TimingError('출발 시각과 base_link 계측 시각이 일치하지 않습니다.')
        gate = self.gates[0]
        _, x, y, _ = self.previous
        if (x-gate['x'])*math.cos(gate['yaw'])+(y-gate['y'])*math.sin(gate['yaw']) >= 0:
            raise TimingError('출발 전에 차량이 이미 출발선을 넘었습니다.')
        self.start_time, self.target_laps, self.active = start_time, laps, True

    def update(self, sample):
        if not all(math.isfinite(v) for v in sample):
            raise TimingError('유효하지 않은 base_link 포즈/시각입니다.')
        previous = self.previous
        if previous is not None and sample[0] < previous[0]:
            raise TimingError('시뮬레이션 시간이 역행했습니다.')
        if previous is not None and sample[0] == previous[0]:
            return []
        self.previous = sample
        if not self.active or previous is None:
            return []
        gap = sample[0]-previous[0]
        self.max_observed_gap = max(self.max_observed_gap, gap)
        if gap > self.max_gap+1e-9:
            raise TimingError(f'계측 데이터 간격 초과: {gap:.6f}초 > {self.max_gap}초')
        if math.dist(sample[1:],previous[1:]) > self.max_step:
            raise TimingError('base_link 위치가 불연속적으로 이동했습니다.')
        events = []
        # Several nearby gates can be crossed by the same sample segment.
        last_time = -math.inf
        for _ in range(len(self.gates)):
            hit = crossing(previous, sample, self.gates[self.next_gate])
            if hit is None or hit['time'] <= last_time:
                break
            last_time = hit['time']
            if hit['time'] < self.start_time:
                break
            hit.update(gate=self.next_gate, lap=len(self.lap_times)+1)
            self.crossings.append(hit)
            events.append(hit)
            if self.next_gate == 0:
                if self.first_crossing is None:
                    self.first_crossing = hit['time']
                else:
                    since = self.last_lap_time if self.last_lap_time is not None else self.first_crossing
                    self.lap_times.append(hit['time']-since)
                    self.last_lap_time = hit['time']
                    if len(self.lap_times) == self.target_laps:
                        self.finish_time, self.finish_bracket = hit['time'], hit['bracket']
                        self.active = False
            self.next_gate = (self.next_gate+1) % len(self.gates)
            if not self.active:
                break
        return events

    def snapshot(self):
        return dict(seen=self.previous is not None, laps=len(self.lap_times),
                    start_time=self.start_time, first_crossing=self.first_crossing,
                    finish_time=self.finish_time, finish_bracket=self.finish_bracket,
                    last_lap_time=self.last_lap_time, lap_times_seconds=list(self.lap_times),
                    approach_seconds=None if self.first_crossing is None else self.first_crossing-self.start_time,
                    next_checkpoint=self.next_gate, max_sample_gap_seconds=self.max_observed_gap)


def rotate(q, p):
    """Quaternion rotate, including nonzero model roll/pitch on slopes."""
    x,y,z,w = q
    n = math.sqrt(sum(v*v for v in q))
    if n < 1e-12:
        raise TimingError('유효하지 않은 회전입니다.')
    x,y,z,w = x/n,y/n,z/n,w/n
    uv = (y*p[2]-z*p[1], z*p[0]-x*p[2], x*p[1]-y*p[0])
    uuv = (y*uv[2]-z*uv[1], z*uv[0]-x*uv[2], x*uv[1]-y*uv[0])
    return tuple(p[i]+2*(w*uv[i]+uuv[i]) for i in range(3))


def world_base_sample(transforms, entry_id, base_link, world):
    """Compose a complete same-timestamp PosePublisher frame chain.

    SceneBroadcaster poses are deliberately not accepted: they have ambiguous
    unscoped link names and lack per-pose timestamps in the ROS bridge.
    """
    edges = {}
    for tf in transforms:
        child = tf.child_frame_id.lstrip('/').replace('::','/')
        if child in edges:
            raise TimingError(f'중복 계측 프레임: {child}')
        edges[child] = tf
    key = f'{entry_id}/{base_link}'
    p, stamps, visited = (0.,0.,0.), set(), set()
    while key not in ('', world):
        if key in visited or key not in edges:
            raise TimingError(f'base_link 프레임 연결 누락: {key}')
        visited.add(key)
        tf = edges[key]; t = tf.transform.translation; q = tf.transform.rotation
        st = tf.header.stamp; stamps.add(st.sec*1_000_000_000+st.nanosec)
        p = tuple(a+b for a,b in zip(rotate((q.x,q.y,q.z,q.w),p),(t.x,t.y,t.z)))
        key = tf.header.frame_id.lstrip('/').replace('::','/')
    if entry_id not in visited or len(stamps) != 1:
        raise TimingError('월드 기준 포즈와 base_link의 시각이 일치하지 않습니다.')
    return (stamps.pop()/1e9, *p)


def duration(seconds):
    if seconds is None:
        return '-'
    millis = round(seconds*1000)
    minutes, rest = divmod(millis, 60_000)
    return f'{minutes:02d}분 {rest/1000:06.3f}초'


def rank_rows(rows):
    """Sort measured times; overlapping measurement brackets need review."""
    rows.sort(key=lambda r: (r['status']!='finished',
              r['time_seconds'] if r['status']=='finished' else -r['laps'],
              r.get('last_lap_elapsed_seconds') if r.get('last_lap_elapsed_seconds') is not None else math.inf,
              r.get('grid_position', 999)))
    group_start, group_hi = 0, -math.inf
    for i, row in enumerate(rows):
        row['rank'], row['rank_provisional'] = i+1, False
        if row['status'] != 'finished':
            continue
        lo, hi = row['elapsed_bracket_seconds']
        if lo <= group_hi:
            row['rank'] = rows[group_start]['rank']
            for other in rows[group_start:i+1]:
                other['rank_provisional'] = True
            group_hi = max(group_hi, hi)
        else:
            group_start, group_hi = i, hi
    return rows
