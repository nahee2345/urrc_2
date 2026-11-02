#!/usr/bin/env python3
"""Select U/I/O/P fixed Gazebo spectator cameras into one ROS image topic.

Press keys in THIS terminal. The rqt viewer remains open; 1..0 remain
reserved for future vehicle-follow cameras and currently do nothing.
"""
import argparse
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import termios
import time
import tty

TRACKS=("monza","hungaroring")
MODES={"u":"start","i":"overview","o":"sector2","p":"sector3"}
OUTPUT="/urrc/spectator/selected/image"


def topic(track,key):
    return f"/urrc/{track}/spectator/{MODES[key]}/image"


def stop(proc):
    if proc is None or proc.poll() is not None:return
    try:
        os.killpg(proc.pid,signal.SIGTERM)
        proc.wait(timeout=3)
    except (ProcessLookupError,subprocess.TimeoutExpired):
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGKILL)
            proc.wait()


def run(track,no_rqt=False):
    import rclpy
    from sensor_msgs.msg import Image
    from rclpy.qos import qos_profile_sensor_data
    selected="u"
    node=None
    bridge=None
    viewer=None
    old_tty=None
    last={}
    try:
        rclpy.init()
        node=rclpy.create_node("urrc_spectator_selector")
        publisher=node.create_publisher(Image,OUTPUT,5)
        subscriptions=[]
        for key in MODES:
            def receive(message,key=key):
                last[key]=time.monotonic()
                if selected==key:
                    publisher.publish(message)
            subscriptions.append(node.create_subscription(
                Image,topic(track,key),receive,qos_profile_sensor_data))
        bridge=subprocess.Popen(["ros2","run","ros_gz_image","image_bridge"]
                                +[topic(track,k) for k in MODES],start_new_session=True)
        if not no_rqt:
            folder=Path(os.environ.get("XDG_STATE_HOME",str(Path.home()/".local/state")))/"urrc_f1"
            folder.mkdir(parents=True,exist_ok=True)
            env=os.environ.copy()
            env.setdefault("QT_QPA_PLATFORM","xcb")
            with (folder/"spectator_rqt.log").open("w") as log:
                viewer=subprocess.Popen(["ros2","run","rqt_image_view","rqt_image_view",OUTPUT],
                    stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        if sys.stdin.isatty():
            old_tty=termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        print(f"[WATCH] {track} U=finish I=whole circuit O=sector2 P=sector3 Q=quit",flush=True)
        print(f"[WATCH] terminal focus is required. Keys 1..0 reserved. Selected topic {OUTPUT}",flush=True)
        updated=0
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.04)
            if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
                key=sys.stdin.read(1).lower()
                if not key or key=="q":break
                if key in MODES:
                    selected=key
                    print(f"[{key.upper()}] {topic(track,key)}",flush=True)
                elif key in "1234567890":
                    print(f"[GRID {key}] reserved for future car-follow cameras",flush=True)
            if time.monotonic()-updated>5:
                seen=[k.upper() for k,t in last.items() if time.monotonic()-t<5]
                print(f"[CAMERAS] selected={selected.upper()} receiving={seen}",flush=True)
                if bridge.poll() is not None:
                    raise RuntimeError(f"ros_gz_image bridge exited: {bridge.returncode}")
                updated=time.monotonic()
    finally:
        if old_tty is not None:
            termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,old_tty)
        stop(viewer)
        stop(bridge)
        if node is not None:node.destroy_node()
        try:
            if rclpy.ok():rclpy.shutdown()
        except (NameError,RuntimeError):pass


def main(argv=None):
    p=argparse.ArgumentParser()
    p.add_argument("track",choices=TRACKS)
    p.add_argument("--no-rqt",action="store_true")
    a=p.parse_args(argv)
    run(a.track,a.no_rqt)


if __name__=="__main__":
    main()
