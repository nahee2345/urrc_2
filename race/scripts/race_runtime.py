"""ROS interfaces for official timing and a persistent common start gate."""
import json
import threading
import time
from timing import LapTimer, TimingError, track_gates, world_base_sample


class PoseLapMonitor:
    def __init__(self, track, root, entries, audit_path, cfg):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('urrc_official_timing')
        self.cv = threading.Condition()
        self.sim_time = None
        self.last_clock_wall = time.monotonic()
        self.last_pose_wall = {}
        self.error = None
        self.track = track
        self.entries = {e['entry_id']:e for e in entries}
        gates = track_gates(root, track, cfg['checkpoint_spacing_m'])
        self.timers = {eid:LapTimer(gates, cfg['max_pose_gap_seconds'], cfg['max_pose_step_m']) for eid in self.entries}
        self.audit = audit_path.open('w', buffering=1)
        self.audit.write(json.dumps({'type':'gates','gates':gates})+'\n')
        self.node.create_subscription(Clock, '/clock', self.clock_cb, 100)
        for eid in self.entries:
            self.node.create_subscription(TFMessage, f'/model/{eid}/urrc_timing',
                                          lambda msg, eid=eid:self.pose_cb(eid,msg), 200)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def clock_cb(self, msg):
        with self.cv:
            value = msg.clock.sec+msg.clock.nanosec/1e9
            if self.sim_time is not None and value < self.sim_time:
                self.error = TimingError('시뮬레이션 시계가 초기화됐습니다.')
            if value != self.sim_time:
                self.last_clock_wall = time.monotonic()
            self.sim_time = value
            self.cv.notify_all()

    def pose_cb(self, eid, msg):
        with self.cv:
            try:
                sample = world_base_sample(msg.transforms, eid, self.entries[eid].get('base_link_name','base_link'), self.track)
                timer = self.timers[eid]
                self.last_pose_wall[eid] = time.monotonic()
                if timer.active:
                    self.audit.write(json.dumps({'type':'pose','entry_id':eid,'sample':sample})+'\n')
                for hit in timer.update(sample):
                    self.audit.write(json.dumps({'type':'crossing','entry_id':eid,**hit})+'\n')
            except Exception as exc:
                self.error = TimingError(f'{eid}: {exc}')
            self.cv.notify_all()

    def check(self):
        if self.error:
            raise self.error

    def wait_seen(self, eid, timeout=30):
        end = time.monotonic()+timeout
        with self.cv:
            while self.timers[eid].previous is None:
                self.check()
                remaining = end-time.monotonic()
                if remaining <= 0:
                    return False
                self.cv.wait(min(.1,remaining))
            return True

    def frozen_time(self, timeout=5):
        """After Gazebo pause acknowledgement, drain clock and pose messages."""
        end = time.monotonic()+timeout
        with self.cv:
            while time.monotonic()<end:
                self.check()
                quiet_since = max([self.last_clock_wall, *self.last_pose_wall.values()])
                if self.sim_time is not None and time.monotonic()-quiet_since >= .25:
                    return self.sim_time
                self.cv.wait(.025)
        raise TimingError('월드 일시정지/계측 동기화에 실패했습니다.')

    def arm(self, ids, laps, start_time):
        with self.cv:
            self.check()
            for eid in ids:
                self.timers[eid].arm(start_time,laps)
                self.audit.write(json.dumps({'type':'arm','entry_id':eid,'start_time':start_time,
                                             'laps':laps,'previous':self.timers[eid].previous})+'\n')

    def snapshot(self, eid):
        with self.cv:
            self.check()
            return self.timers[eid].snapshot()

    def deactivate(self, eid):
        with self.cv:
            self.timers[eid].active = False

    def health(self, ids, wall_timeout=5):
        with self.cv:
            self.check()
            now = time.monotonic()
            if now-self.last_clock_wall > wall_timeout:
                raise TimingError('시뮬레이션 시계가 멈췄습니다. 이번 기록은 무효입니다.')
            for eid in ids:
                timer = self.timers[eid]
                if not timer.active:
                    continue
                if now-self.last_pose_wall.get(eid,0) > wall_timeout:
                    raise TimingError(f'{eid}: base_link 계측이 중단됐습니다.')
                # Check timestamp gaps when another pose arrives; /clock and pose
                # callbacks can have different transport delays.

    def note_resume(self):
        # Controller cleanup may take seconds of wall time with physics frozen.
        # Do not treat an intentional operator pause as transport failure.
        with self.cv:
            now = time.monotonic()
            self.last_clock_wall = now
            for eid, timer in self.timers.items():
                if timer.active:
                    self.last_pose_wall[eid] = now

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=3)
        self.node.destroy_node()
        self.audit.close()


class StartGate:
    def __init__(self, entries):
        import rclpy
        from std_msgs.msg import Bool
        if not rclpy.ok():
            rclpy.init()
        self.rclpy, self.Bool = rclpy, Bool
        self.node = rclpy.create_node('urrc_start_gate')
        self.pubs = {e['entry_id']:self.node.create_publisher(Bool,f"/race/start/{e['entry_id']}",10) for e in entries}

    def ready(self, ids, processes, timeout=30):
        end = time.monotonic()+timeout
        while time.monotonic()<end:
            if all(self.pubs[eid].get_subscription_count()>0 and processes[eid].poll() is None for eid in ids):
                return []
            self.rclpy.spin_once(self.node, timeout_sec=.05)
        return [eid for eid in ids if self.pubs[eid].get_subscription_count()==0 or processes[eid].poll() is not None]

    def publish(self, ids, value):
        from rclpy.duration import Duration
        msg = self.Bool(); msg.data = value
        for eid in ids:
            self.pubs[eid].publish(msg)
        # Publishers were discovered before the light sequence. Publish all
        # commands before waiting for ACKs, while physics is paused at start.
        for eid in ids:
            if not self.pubs[eid].wait_for_all_acked(Duration(seconds=2)):
                raise TimingError(f'{eid}: 공통 출발/정지 신호 전송 확인 실패')
        self.rclpy.spin_once(self.node, timeout_sec=.05)

    def close(self):
        self.node.destroy_node()
