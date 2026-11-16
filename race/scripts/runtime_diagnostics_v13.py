"""Read-only URRC race-side diagnostics; never changes participant controller state.

Used by race_manager.PoseLapMonitor wrapper. ROS 2 message observations are
best-effort diagnostics and are NEVER treated as proof of camera recognition.
"""
from __future__ import annotations
import math
import time


def classify(state, prefix='제한시간 초과'):
    """Classify the last observable stage; not a claim about source-code causation."""
    speeds = int(state.get('speed_messages_after_release') or 0)
    steers = int(state.get('steering_messages_after_release') or 0)
    max_speed = float(state.get('max_abs_speed_command') or 0.0)
    moved = float(state.get('max_displacement_m') or 0.0)
    if not speeds or not steers:
        detail = f'ROS 제어 명령 미수신(speed={speeds}, steering={steers}); launch/토픽/제어기 확인'
    elif max_speed <= 0.05:
        detail = f'속도 명령이 0 유지(max={max_speed:.3f}m/s); 카메라·출발 허가·차량 안전 조건 확인'
    elif moved <= 0.05:
        detail = f'속도 명령(max={max_speed:.3f}m/s)은 있으나 base_link 정지({moved:.3f}m); 브리지/구동/물리 확인'
    elif state.get('first_crossing') is None:
        detail = f'차량 이동({moved:.3f}m) 확인, 최초 출발선 통과 없음'
    else:
        detail = f'출발선 통과 후 랩 미완료(laps={state.get("laps",0)}, 다음 체크포인트={state.get("next_checkpoint")})'
    return prefix + '; ' + detail


def format_line(eid, state):
    return (f'{eid}: 속도={state.get("last_speed_command")} '
            f'(최대={float(state.get("max_abs_speed_command") or 0):.3f}m/s, '
            f'수신={state.get("speed_messages_after_release", 0)}), '
            f'조향={state.get("last_steering_command")} '
            f'(수신={state.get("steering_messages_after_release", 0)}), '
            f'base_link 이동={float(state.get("max_displacement_m") or 0):.3f}m, '
            f'랩={state.get("laps", 0)}, 다음 게이트={state.get("next_checkpoint", "-")}')


def wrap_monitor_v13(base):
    """Add diagnostics only when absent; initialize ROS without SIGINT auto-shutdown.

    This is safe for installations where Codex has already added diagnostics.
    """
    has_existing = callable(getattr(base, 'diagnostic_line', None)) and callable(getattr(base, 'failure_reason', None))

    class V13Monitor(base):
        def __init__(self, *args, **kwargs):
            import rclpy
            if not rclpy.ok():
                from rclpy.signals import SignalHandlerOptions
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            if not has_existing:
                self._v13_stats = {}
                self._v13_armed = set()
                self._v13_origin = {}
                self._v13_handles = []
            super().__init__(*args, **kwargs)
            if not has_existing:
                from std_msgs.msg import Float32
                for eid, entry in self.entries.items():
                    self._v13_stats[eid] = dict(
                        speed_count=0, steering_count=0,
                        speed_messages_after_release=0,
                        steering_messages_after_release=0,
                        last_speed_command=None, last_steering_command=None,
                        max_abs_speed_command=0., max_abs_steering_command=0.,
                        max_displacement_m=0., observed_after_release_seconds=None)
                    for kind, topic in (('speed', entry['speed_topic']), ('steering', entry['steering_topic'])):
                        self._v13_handles.append(
                            self.node.create_subscription(Float32, topic,
                                lambda msg, team=eid, kind=kind: self._v13_cmd(team, kind, msg), 50))

        def _v13_cmd(self, eid, kind, msg):
            if has_existing:
                return
            with self.cv:
                stat = self._v13_stats.get(eid)
                if stat is None:
                    return
                value = float(msg.data)
                stat[f'{kind}_count'] += 1
                stat[f'last_{kind}_command'] = value if math.isfinite(value) else None
                if eid in self._v13_armed:
                    stat[f'{kind}_messages_after_release'] += 1
                    if math.isfinite(value):
                        key=f'max_abs_{kind}_command'
                        stat[key] = max(stat[key], abs(value))

        def pose_cb(self, eid, msg):
            super().pose_cb(eid, msg)
            if has_existing or not hasattr(self, '_v13_stats'):
                return
            with self.cv:
                if eid not in self._v13_armed or eid not in self._v13_origin:
                    return
                prev = self.timers[eid].previous
                if prev is None or len(prev) < 4:
                    return
                origin = self._v13_origin[eid]
                if len(origin) != 3:
                    return
                distance = math.dist(prev[1:4], origin)
                if math.isfinite(distance):
                    self._v13_stats[eid]['max_displacement_m'] = max(
                        distance, self._v13_stats[eid]['max_displacement_m'])

        def arm(self, ids, laps, start_time):
            super().arm(ids, laps, start_time)
            if has_existing:
                return
            with self.cv:
                for eid in ids:
                    previous = self.timers[eid].previous
                    if previous is not None and len(previous) >= 4:
                        self._v13_origin[eid] = tuple(previous[1:4])
                    stat = self._v13_stats[eid]
                    for kind in ('speed', 'steering'):
                        stat[f'{kind}_messages_after_release'] = 0
                        stat[f'max_abs_{kind}_command'] = 0.
                    stat['max_displacement_m'] = 0.
                    self._v13_armed.add(eid)

        def deactivate(self, eid):
            if not has_existing:
                with self.cv:
                    self._v13_armed.discard(eid)
            return super().deactivate(eid)

        def snapshot(self, eid):
            result=super().snapshot(eid)
            if not has_existing:
                with self.cv:
                    result.update(self._v13_stats[eid])
                    start = result.get('start_time')
                    previous = self.timers[eid].previous
                    result['observed_after_release_seconds'] = (
                        max(0., previous[0]-start) if start is not None and previous is not None else None)
            return result

        def failure_reason(self, eid, prefix):
            if has_existing:
                return super().failure_reason(eid, prefix)
            return classify(self.snapshot(eid), prefix)

        def diagnostic_line(self, eid):
            if has_existing:
                return super().diagnostic_line(eid)
            return format_line(eid, self.snapshot(eid))

    V13Monitor.__name__='V13PoseLapMonitor'
    return V13Monitor
