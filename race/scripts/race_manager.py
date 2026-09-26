#!/usr/bin/env python3
"""URRC intake and F1-style qualifying/final orchestration for Gazebo Sim."""
from __future__ import annotations
import argparse, csv, json, math, os, random, re, shutil, signal, subprocess, sys, tempfile, threading, time, uuid, zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path, PurePosixPath
ENTRY_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{0,31}$")
MAX_ENTRIES = 12


def read_json(path): return json.loads(Path(path).read_text())
def lap_crossing(previous, current, lateral, half_width): return previous is not None and previous < 0 <= current and abs(lateral) <= half_width

def safe_extract(archive, destination):
    for member in archive.infolist():
        rel = PurePosixPath(member.filename)
        if rel.is_absolute() or ".." in rel.parts or not rel.parts: raise ValueError(f"ZIP 경로 오류: {member.filename}")
        target = (destination / Path(*rel.parts)).resolve()
        if destination.resolve() not in target.parents and target != destination.resolve(): raise ValueError(f"ZIP 경로 오류: {member.filename}")
        if member.is_dir(): target.mkdir(parents=True, exist_ok=True)
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(member) as src, target.open("wb") as dst: shutil.copyfileobj(src, dst)

def load_entries(root, maximum=MAX_ENTRIES):
    entries=[]; base=root/"race/entrants"
    for folder in sorted(base.iterdir()):
        mf=folder/"race_entry.json"
        if not folder.is_dir() or not mf.is_file(): continue
        e=read_json(mf); required=("entry_id","display_name","ros_package","algorithm_launch_file","vehicle_model_file","speed_topic","steering_topic","lidar_topic")
        missing=[k for k in required if not e.get(k)]
        if missing: raise ValueError(f"{mf}: 필수 항목 누락 {missing}")
        if not ENTRY_RE.fullmatch(e["entry_id"]) or e["entry_id"]!=folder.name: raise ValueError(f"{mf}: entry_id는 참가 폴더명과 같아야 합니다.")
        if e.get("vehicle_model_format","sdf").lower() not in ("sdf","urdf"): raise ValueError(f"{mf}: vehicle_model_format은 sdf/urdf만 가능합니다.")
        model=(folder/e["vehicle_model_file"]).resolve()
        if folder.resolve() not in model.parents or not model.is_file(): raise ValueError(f"{mf}: 차량 모델 경로가 없거나 참가 폴더 밖입니다.")
        e["_folder"]=str(folder.resolve()); e["_model"]=str(model); entries.append(e)
    if len(entries)>maximum: raise ValueError(f"참가자는 최대 {maximum}개까지 가능합니다. 현재 {len(entries)}개입니다.")
    for key in ("entry_id","ros_package","speed_topic","steering_topic","lidar_topic"):
        vals=[e[key] for e in entries]
        if len(vals)!=len(set(vals)): raise ValueError(f"{key}은 참가자마다 고유해야 합니다.")
    for e in entries:
        for key in ("speed_topic","steering_topic","lidar_topic"):
            if not e[key].startswith("/"): raise ValueError(f"{e['entry_id']}의 {key}은 절대 ROS 토픽 이름이어야 합니다.")
    return entries

def intake(root):
    inbox=root/"race/inbox"; entries=root/"race/entrants"; inbox.mkdir(parents=True,exist_ok=True); entries.mkdir(parents=True,exist_ok=True)
    for zpath in sorted(inbox.glob("*.zip")):
        temp=Path(tempfile.mkdtemp(prefix="entry_",dir=root/"race"))
        try:
            with zipfile.ZipFile(zpath) as z: safe_extract(z,temp)
            mf=temp/"race_entry.json"
            if not mf.is_file(): raise ValueError(f"{zpath.name}: ZIP 최상위에 race_entry.json이 없습니다.")
            entry=read_json(mf); eid=entry.get("entry_id","")
            if not ENTRY_RE.fullmatch(eid): raise ValueError(f"{zpath.name}: entry_id 형식 오류")
            if not (temp/"src").is_dir(): raise ValueError(f"{zpath.name}: ROS 패키지가 포함된 src/ 폴더가 필요합니다.")
            target=entries/eid
            if target.exists(): raise ValueError(f"{eid}: 기존 참가 폴더가 있어 덮어쓰지 않았습니다.")
            temp.rename(target); zpath.rename(zpath.with_suffix(".loaded")); print(f"접수 {eid}: {zpath.name}")
        finally:
            if temp.exists(): shutil.rmtree(temp,ignore_errors=True)
    for source in sorted(p for p in inbox.iterdir() if p.is_dir()):
        mf=source/"race_entry.json"
        if not mf.is_file():
            raise ValueError(f"{source.name}: 폴더 최상위에 race_entry.json이 없습니다.")
        entry=read_json(mf); eid=entry.get("entry_id","")
        if not ENTRY_RE.fullmatch(eid) or eid != source.name:
            raise ValueError(f"{source.name}: 폴더명과 유효한 entry_id가 같아야 합니다.")
        if not (source/"src").is_dir():
            raise ValueError(f"{source.name}: ROS 패키지가 포함된 src/ 폴더가 필요합니다.")
        target=entries/eid
        if target.exists(): raise ValueError(f"{eid}: 기존 참가 폴더가 있어 덮어쓰지 않았습니다.")
        shutil.move(str(source),str(target)); print(f"접수 {eid}: {source.name}/")
    return load_entries(root)

def get_event(root,new=False):
    cfg=read_json(root/"race/event_config.json"); path=root/"race/event_state.json"
    if new or not path.exists():
        event={"event_id":uuid.uuid4().hex[:10],"track":"monza","created_unix":time.time(),"seed":random.randrange(1,2**31)}
        path.write_text(json.dumps(event,indent=2)+"\n"); return event
    event=read_json(path)
    if event["track"] not in cfg["track_pool"]: raise ValueError("대회 트랙 설정 오류: new-event로 새로 추첨하세요.")
    return event

def launch_args(e):
    args=dict(e.get("launch_arguments",{})); args.update({"race_start_topic":f"/race/start/{e['entry_id']}","speed_topic":e["speed_topic"],"steering_topic":e["steering_topic"],"lidar_topic":e["lidar_topic"],"use_sim_time":"true"})
    return [f"{k}:={v}" for k,v in sorted(args.items())]

class PoseLapMonitor:
    def __init__(self,track,root,ids):
        import rclpy
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        self.rclpy=rclpy
        if not rclpy.ok(): rclpy.init(args=None)
        self.node=Node("urrc_race_lap_monitor"); self.cv=threading.Condition(); self.sim_time=0.
        self.states={i:{"seen":False,"previous_s":None,"laps":0,"armed":False,"start_time":None,"finish_time":None,"last_lap_time":None} for i in ids}
        meta=read_json(root/f"src/urrc_track_gazebo/tracks/processed/{track}.json")["start_finish"]
        self.x0,self.y0,self.yaw=meta["x"],meta["y"],meta["yaw"]
        with (root/f"src/urrc_track_gazebo/tracks/processed/{track}_centerline.csv").open() as f: width=float(next(csv.DictReader(f))["road_width_m"])
        self.half_width=width/2+.15
        self.node.create_subscription(Clock,"/clock",self.clock_cb,20)
        self.node.create_subscription(TFMessage,f"/world/{track}/dynamic_pose/info",self.pose_cb,100)
        self.executor=rclpy.executors.SingleThreadedExecutor(); self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True); self.thread.start()
    def clock_cb(self,msg):
        with self.cv: self.sim_time=msg.clock.sec+msg.clock.nanosec*1e-9; self.cv.notify_all()
    def pose_cb(self,msg):
        with self.cv:
            for tf in msg.transforms:
                frame=tf.child_frame_id.lstrip("/")
                for name,s in self.states.items():
                    # Count the top-level model pose only; link frames may straddle
                    # the timing plane on adjacent updates and would double-count.
                    if frame!=name: continue
                    s["seen"]=True; t=tf.transform.translation; dx,dy=t.x-self.x0,t.y-self.y0
                    progress=dx*math.cos(self.yaw)+dy*math.sin(self.yaw); lateral=-dx*math.sin(self.yaw)+dy*math.cos(self.yaw)
                    if s["armed"] and lap_crossing(s["previous_s"],progress,lateral,self.half_width):
                        st=tf.header.stamp; now=st.sec+st.nanosec*1e-9 or self.sim_time
                        s["laps"]+=1; s["last_lap_time"]=now
                        if s["laps"]>=s["target_laps"]: s["finish_time"]=now
                    s["previous_s"]=progress
            self.cv.notify_all()
    def wait_seen(self,name,timeout=30):
        end=time.monotonic()+timeout
        with self.cv:
            while not self.states[name]["seen"]:
                remain=end-time.monotonic()
                if remain<=0:return False
                self.cv.wait(min(.25,remain))
            return True
    def arm(self,names,laps):
        with self.cv:
            for n in names:
                s=self.states[n]; s.update({"laps":0,"armed":True,"target_laps":laps,"start_time":self.sim_time,"finish_time":None,"last_lap_time":None,"previous_s":None})
    def snapshot(self,n):
        with self.cv:return dict(self.states[n])
    def close(self):
        self.executor.shutdown(); self.node.destroy_node(); self.rclpy.shutdown()

def publish_start_batch(entrants,value,timeout=30):
    # Use one ROS node so all ten race-start messages are emitted together.
    import rclpy
    from std_msgs.msg import Bool
    if not rclpy.ok(): rclpy.init(args=None)
    node=rclpy.create_node("urrc_race_start_gate")
    topics=[f"/race/start/{e['entry_id']}" for e in entrants]
    pubs=[node.create_publisher(Bool,t,10) for t in topics]
    deadline=time.monotonic()+timeout
    try:
        while time.monotonic()<deadline and not all(pub.get_subscription_count()>0 for pub in pubs):
            rclpy.spin_once(node,timeout_sec=.1)
        if not all(pub.get_subscription_count()>0 for pub in pubs):
            raise TimeoutError("모든 참가 알고리즘이 race_start_topic을 구독하지 않았습니다.")
        msg=Bool();msg.data=bool(value)
        for pub in pubs: pub.publish(msg)
        time.sleep(.05)
        rclpy.spin_once(node,timeout_sec=.05)
    finally:
        for pub in pubs: node.destroy_publisher(pub)
        node.destroy_node()
def light_request(track,index,on,root):
    # Change the lens material itself so the rectangular LED glows without
    # washing red light across the gantry and track.
    emissive=("r: 1 g: 0.008 b: 0.008 a: 1" if on
              else "r: 0 g: 0 b: 0 a: 1")
    req=(f'name: "red_lens_{index+1}" parent_name: "gantry_visuals" '
         'material { ambient { r: 0.12 g: 0.006 b: 0.008 a: 1 } '
         'diffuse { r: 0.12 g: 0.006 b: 0.008 a: 1 } '
         'specular { r: 0.02 g: 0.02 b: 0.02 a: 1 } '
         f'emissive {{ {emissive} }} }}')
    subprocess.run(["gz","service","-s",f"/world/{track}/visual_config","--reqtype","gz.msgs.Visual","--reptype","gz.msgs.Boolean","--timeout","1500","--req",req],check=True,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,timeout=5)
def lights_all(track,root,on):
    with ThreadPoolExecutor(max_workers=5) as pool:
        futures=[pool.submit(light_request,track,i,on,root) for i in range(5)]
        for future in futures: future.result()
def start_sequence(track,root,entrants,delay):
    publish_start_batch(entrants,False)
    for i in range(5): light_request(track,i,True,root); time.sleep(1.0)
    time.sleep(delay)
    lights_all(track,root,False)

def start_world(root,track,event):
    pkg=root/"src/urrc_track_gazebo"; env=os.environ.copy()
    resource_paths=[str(pkg/"models")]
    for entry_dir in (root/"race/entrants").iterdir():
        source=entry_dir/"src"
        if source.is_dir():
            resource_paths.append(str(source))
            resource_paths.extend(str(p) for p in source.rglob("models") if p.is_dir())
    inherited=env.get("GZ_SIM_RESOURCE_PATH","")
    if inherited: resource_paths.append(inherited)
    env["GZ_SIM_RESOURCE_PATH"]=os.pathsep.join(resource_paths)
    env["GZ_PARTITION"]="urrc_race_"+event["event_id"]
    log=(root/"race/results/gazebo.log").open("w")
    server=subprocess.Popen(["gz","sim","-r","--gui-config",str(pkg/"worlds"/f"{track}_grid.gui.config"),str(pkg/"worlds"/f"{track}_race.sdf")],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    for _ in range(90):
        if server.poll() is not None:raise RuntimeError("Gazebo가 시작 직후 종료됐습니다. race/results/gazebo.log 확인")
        p=subprocess.run(["gz","service","-l"],capture_output=True,text=True,timeout=3)
        if f"/world/{track}/visual_config" in p.stdout:break
        time.sleep(.5)
    else:raise TimeoutError("Gazebo의 신호등 제어 서비스를 찾지 못했습니다.")
    bridge=subprocess.Popen(["ros2","run","ros_gz_bridge","parameter_bridge","/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",f"/world/{track}/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,start_new_session=True)
    return server,bridge,env,log

def stop(proc):
    if proc and proc.poll() is None:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGTERM)
            try:proc.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
def spawn(root,track,e,slot,env):
    z=slot["road_z"]+float(e.get("spawn_z_offset_m",.04))
    subprocess.run(["ros2","run","ros_gz_sim","create","--world",track,"--file",e["_model"],"--name",e["entry_id"],"--x",str(slot["x"]),"--y",str(slot["y"]),"--z",str(z),"--yaw",str(slot["yaw"])],check=True,env=env,timeout=60)
def remove(track,eid,env):
    subprocess.run(["ros2","run","ros_gz_sim","remove","--world",track,"--entity",eid],env=env,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=30)
def launch_algo(e,env):
    cmd=["ros2","launch",e["ros_package"],e["algorithm_launch_file"],*launch_args(e)]
    return subprocess.Popen(cmd,env=env,stdout=subprocess.DEVNULL,stderr=subprocess.STDOUT,start_new_session=True)
def validate(root):
    es=load_entries(root)
    for e in es:
        p=subprocess.run(["ros2","pkg","prefix",e["ros_package"]],capture_output=True,text=True)
        if p.returncode:raise ValueError(f"ROS 패키지를 빌드하지 못했습니다: {e['ros_package']}")
        launch=Path(p.stdout.strip())/"share"/e["ros_package"]/"launch"/e["algorithm_launch_file"]
        if not launch.is_file():raise ValueError(f"launch 파일 없음: {launch}")
    import xml.etree.ElementTree as ET
    package=root/"src/urrc_track_gazebo"
    for track in ("monza",):
        world=ET.parse(package/"worlds"/f"{track}_race.sdf").getroot()
        gantry=world.find(".//model[@name='race_start_gantry']")
        lenses=gantry.findall("./link[@name='gantry_visuals']/visual") if gantry is not None else []
        lens_names={n.get("name") for n in lenses}
        lights=[n for n in world.findall("world/light") if n.get("name","").startswith("race_red_")]
        meta=read_json(package/f"tracks/processed/{track}.json")
        expected={f"red_lens_{i}" for i in range(1,6)}
        if gantry is None or not expected.issubset(lens_names) or len(lights)!=5 or len(meta["grid"])!=20:
            raise ValueError(f"{track}: 신호등 렌즈/그리드 리소스가 완전하지 않습니다.")
        if any(float(n.findtext("intensity","-1"))!=0 for n in lights):
            raise ValueError(f"{track}: 신호등 보조 광원은 대기 시 모두 꺼져 있어야 합니다.")
    print(f"참가 패키지 {len(es)}개 검사 PASS (최대 {MAX_ENTRIES}개)");return es
def grid(root,track):return read_json(root/f"src/urrc_track_gazebo/tracks/processed/{track}.json")["grid"]
def print_table(rows,title):
    print(f"\n{title}\n순위 | 참가자 | 패키지 | 결과")
    for i,r in enumerate(rows,1):print(f"{i:>2} | {r['display_name']} | {r['ros_package']} | {f'{r['time_seconds']:.3f}초' if r.get('status')=='finished' else 'DNF'}")
def run_mode(root,mode):
    cfg=read_json(root/"race/event_config.json");entries=validate(root);event=get_event(root);track=event["track"]
    print(f"선택 트랙: {track} / 예선·본선 동일")
    if not entries:
        print("등록된 참가 패키지가 없어 이번 실행은 건너뜁니다. 패키지를 추가한 뒤 다시 실행하세요.")
        return
    results=root/"race/results";results.mkdir(exist_ok=True)
    if mode=="final":
        qpath=results/"qualifying.json"
        if not qpath.is_file():raise ValueError("예선을 먼저 완료하세요.")
        q=read_json(qpath);rank={r["entry_id"]:i for i,r in enumerate(q["results"])};entries.sort(key=lambda e:rank[e["entry_id"]])
        laps,timeout=int(cfg["final_laps"]),int(cfg["race_timeout_seconds"])
    else:laps,timeout=int(cfg["qualifying_laps"]),int(cfg["lap_timeout_seconds"])
    slots=grid(root,track);server=bridge=monitor=None;algos=[];out=[]
    try:
        server,bridge,env,log=start_world(root,track,event);time.sleep(3)
        monitor=PoseLapMonitor(track,root,[e["entry_id"] for e in entries]);rng=random.Random(event["seed"])
        if mode=="qualifying":
            for e in entries:
                print(f"예선 주행: {e['display_name']}")
                spawn(root,track,e,slots[0],env)
                if not monitor.wait_seen(e["entry_id"]):raise TimeoutError("차량 포즈를 읽지 못했습니다. ros_gz_bridge/Gazebo 동작을 확인하세요.")
                proc=launch_algo(e,env)
                try:
                    start_sequence(track,root,[e],rng.uniform(cfg["lights_out_min_delay_seconds"],cfg["lights_out_max_delay_seconds"]))
                    monitor.arm([e["entry_id"]],laps);publish_start_batch([e],True)
                    end=time.monotonic()+timeout
                    while time.monotonic()<end:
                        s=monitor.snapshot(e["entry_id"])
                        if s["laps"]>=laps or proc.poll() is not None:break
                        time.sleep(.1)
                    s=monitor.snapshot(e["entry_id"]);done=s["laps"]>=laps
                    out.append({"entry_id":e["entry_id"],"display_name":e["display_name"],"ros_package":e["ros_package"],"status":"finished" if done else "DNF","laps":s["laps"],"time_seconds":max(0,s["last_lap_time"]-s["start_time"]) if done else None})
                finally:stop(proc);remove(track,e["entry_id"],env);lights_all(track,root,False)
            out.sort(key=lambda r:(r["status"]!="finished",r["time_seconds"] if r["time_seconds"] is not None else float("inf")))
            (results/"qualifying.json").write_text(json.dumps({"event":event,"track":track,"laps":laps,"results":out},indent=2)+"\n");print_table(out,"예선 순위")
        else:
            for e,slot in zip(entries,slots):
                spawn(root,track,e,slot,env)
                if not monitor.wait_seen(e["entry_id"]):raise TimeoutError(f"차량 포즈를 읽지 못했습니다: {e['entry_id']}")
                algos.append((e,launch_algo(e,env)))
            start_sequence(track,root,entries,rng.uniform(cfg["lights_out_min_delay_seconds"],cfg["lights_out_max_delay_seconds"]))
            monitor.arm([e["entry_id"] for e in entries],laps)
            publish_start_batch(entries,True)
            deadline=time.monotonic()+timeout;reported=set();finish=[];dnf=[]
            while time.monotonic()<deadline and len(reported)<len(entries):
                for e,proc in algos:
                    eid=e["entry_id"]
                    if eid in reported:continue
                    s=monitor.snapshot(eid)
                    if s["laps"]>=laps:
                        row={"entry_id":eid,"display_name":e["display_name"],"ros_package":e["ros_package"],"status":"finished","laps":s["laps"],"time_seconds":max(0,s["last_lap_time"]-s["start_time"])}
                        row["finish_sim_time"]=s["finish_time"]
                        finish.append(row);reported.add(eid)
                    elif proc.poll() is not None:
                        dnf.append({"entry_id":eid,"display_name":e["display_name"],"ros_package":e["ros_package"],"status":"DNF","laps":s["laps"],"time_seconds":None});reported.add(eid)
                time.sleep(.1)
            for e,_ in algos:
                if e["entry_id"] not in reported:
                    s=monitor.snapshot(e["entry_id"]);dnf.append({"entry_id":e["entry_id"],"display_name":e["display_name"],"ros_package":e["ros_package"],"status":"DNF","laps":s["laps"],"time_seconds":None})
            finish.sort(key=lambda r:r["finish_sim_time"])
            for i,row in enumerate(finish,1): print(f"완주 {i}위: {row['display_name']} ({row['ros_package']}) / {row['time_seconds']:.3f}s")
            dnf.sort(key=lambda r:r["laps"],reverse=True);out=finish+dnf
            (results/"final.json").write_text(json.dumps({"event":event,"track":track,"laps":laps,"results":out},indent=2)+"\n")
            with (results/"final.csv").open("w",newline="") as f:
                w=csv.DictWriter(f,fieldnames=["rank","entry_id","display_name","ros_package","status","laps","time_seconds","finish_sim_time"]);w.writeheader()
                for i,r in enumerate(out,1):w.writerow({"rank":i,**r})
            print_table(out,"본선 최종 순위")
    finally:
        for _,proc in algos:stop(proc)
        if monitor:monitor.close()
        if bridge:stop(bridge)
        if server:stop(server)
        if 'log' in locals():log.close()

def main():
    p=argparse.ArgumentParser();p.add_argument("command",choices=("intake","validate","new-event","qualifying","final","status"));p.add_argument("--root",type=Path,required=True);a=p.parse_args();root=a.root.resolve()
    try:
        if a.command=="intake":print(f"접수 완료: {len(intake(root))}개 참가 패키지")
        elif a.command=="validate":validate(root)
        elif a.command=="new-event":
            e=get_event(root,True)
            for f in (root/"race/results").glob("*.json"):f.unlink()
            (root/"race/results/final.csv").unlink(missing_ok=True);print(f"새 이벤트 {e['event_id']} / 추첨 트랙 {e['track']}")
        elif a.command in ("qualifying","final"):run_mode(root,a.command)
        else:
            ep=root/"race/event_state.json";print(json.dumps(read_json(ep) if ep.is_file() else {"event":"미생성"},ensure_ascii=False,indent=2))
            for f in (root/"race/results/qualifying.json",root/"race/results/final.json"):
                if f.is_file():print(f"{f.name}: {len(read_json(f)['results'])}개 결과")
    except Exception as exc:print(f"대회 실행 오류: {exc}",file=sys.stderr);return 2
    return 0
if __name__=="__main__":sys.exit(main())
