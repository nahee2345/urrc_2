#!/usr/bin/env python3
"""URRC intake and F1-style qualifying/final orchestration for Gazebo Sim."""
from __future__ import annotations
import argparse, csv, hashlib, json, math, os, random, re, shutil, signal, subprocess, sys, tempfile, time, uuid, zipfile
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path, PurePosixPath
ENTRY_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_-]{0,31}$")
MAX_ENTRIES = 12
sys.path.insert(0,str(Path(__file__).resolve().parent))
from submission import normalize
from timing import TimingError, duration, rank_rows
from race_runtime import PoseLapMonitor, StartGate


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
            if (member.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError(f"ZIP 심볼릭 링크는 허용되지 않습니다: {member.filename}")
            with archive.open(member) as src, target.open("wb") as dst: shutil.copyfileobj(src, dst)
            mode=(member.external_attr >> 16) & 0o777
            if mode:target.chmod(mode)

def load_entries(root, maximum=MAX_ENTRIES):
    entries=[]; base=root/"race/entrants"; base.mkdir(parents=True,exist_ok=True)
    for folder in sorted(base.iterdir()):
        mf=folder/"race_entry.json"
        if not folder.is_dir() or not mf.is_file(): continue
        e=read_json(mf); required=("entry_id","display_name","ros_package","algorithm_launch_file","vehicle_model_file","speed_topic","steering_topic","lidar_topic")
        missing=[k for k in required if not e.get(k)]
        if missing: raise ValueError(f"{mf}: 필수 항목 누락 {missing}")
        if not ENTRY_RE.fullmatch(e["entry_id"]) or e["entry_id"]!=folder.name: raise ValueError(f"{mf}: entry_id는 참가 폴더명과 같아야 합니다.")
        if e.get("vehicle_model_format","sdf").lower() not in ("sdf","urdf"): raise ValueError(f"{mf}: vehicle_model_format은 sdf/urdf만 가능합니다.")
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*',e.get('base_link_name','base_link')):
            raise ValueError(f'{mf}: base_link_name은 최상위 실제 링크 이름이어야 합니다.')
        if not math.isfinite(float(e.get('spawn_z_offset_m',.04))):
            raise ValueError(f'{mf}: 유효하지 않은 스폰 높이입니다.')
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
    inbox=root/"race/inbox"; entries=root/"race/entrants"
    inbox.mkdir(parents=True,exist_ok=True); entries.mkdir(parents=True,exist_ok=True)
    sources=sorted(p for p in inbox.iterdir() if not p.name.startswith('.') and not p.name.endswith('.loaded') and (p.is_dir() or p.suffix=='.zip'))
    for source in sources:
        with tempfile.TemporaryDirectory(prefix="intake_",dir=root/"race") as d:
            temp=Path(d); raw=source
            if source.is_file():
                raw=temp/'raw'; raw.mkdir()
                with zipfile.ZipFile(source) as z:safe_extract(z,raw)
            stage=temp/'entry'; eid=normalize(raw,stage)
            target=entries/eid
            if target.exists():raise ValueError(f"{eid}: 기존 참가 폴더가 있어 덮어쓰지 않았습니다.")
            stage.rename(target)
            source.rename(source.with_name(source.name+'.loaded'))
            print(f"접수 {eid}: {source.name}")
    return load_entries(root)

def get_event(root,new=False,track=None):
    cfg=read_json(root/"race/event_config.json"); path=root/"race/event_state.json"
    if track is not None and track not in cfg["track_pool"]:
        raise ValueError(f"선택할 수 없는 트랙: {track}")
    if new or not path.exists():
        event={"event_id":uuid.uuid4().hex[:10],"track":track or random.choice(cfg["track_pool"]),"created_unix":time.time(),"seed":random.randrange(1,2**31)}
        path.write_text(json.dumps(event,indent=2)+"\n"); return event
    event=read_json(path)
    if event["track"] not in cfg["track_pool"]: raise ValueError("대회 트랙 설정 오류: new-event로 새로 추첨하세요.")
    return event

def launch_args(e):
    args=dict(e.get("launch_arguments",{})); args.update({"race_start_topic":f"/race/start/{e['entry_id']}","speed_topic":e["speed_topic"],"steering_topic":e["steering_topic"],"lidar_topic":e["lidar_topic"],"use_sim_time":"true","race_start_mode":e.get("_start_mode","signal")})
    return [f"{k}:={v}" for k,v in sorted(args.items())]

def gz_service(track, service, reqtype, request, env=None):
    result=subprocess.run(["gz","service","-s",f"/world/{track}/{service}",
        "--reqtype",reqtype,"--reptype","gz.msgs.Boolean","--timeout","5000","--req",request],
        check=True,capture_output=True,text=True,timeout=8,env=env)
    if not re.search(r"data:\s*true", result.stdout):
        raise RuntimeError(f"Gazebo {service} 요청 실패: {result.stdout} {result.stderr}")


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
    gz_service(track,"visual_config","gz.msgs.Visual",req)
def lights_all(track,root,on):
    with ThreadPoolExecutor(max_workers=5) as pool:
        futures=[pool.submit(light_request,track,i,on,root) for i in range(5)]
        for future in futures: future.result()
def pause_world(track, paused, env):
    gz_service(track, "control", "gz.msgs.WorldControl", "pause: "+str(paused).lower(), env)


def start_sequence(track, root, entries, delay, monitor, gate, laps, env):
    ids = [e['entry_id'] for e in entries]
    gate.publish(ids, False)
    lights_all(track, root, False)
    for i in range(5):
        light_request(track, i, True, root)
        time.sleep(1.)
    time.sleep(delay)
    pause_world(track, True, env)
    start_time = monitor.frozen_time()
    monitor.arm(ids, laps, start_time)
    # Signal mode: common release. Camera mode: permit camera-controlled start.
    # Camera nodes must first observe red, then all red lights off.
    gate.publish(ids, True)
    lights_all(track, root, False)
    time.sleep(.1)
    pause_world(track, False, env)
    monitor.note_resume()
    print(f"공통 출발: sim={start_time:.9f}초", flush=True)
    return start_time

def start_world(root,track,event,entries,run_dir):
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
    os.environ["GZ_PARTITION"]=env["GZ_PARTITION"]
    log=(run_dir/"gazebo.log").open("w")
    server=subprocess.Popen(["gz","sim","-r","--gui-config",str(pkg/"worlds"/f"{track}_grid.gui.config"),str(pkg/"worlds"/f"{track}_race.sdf")],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        for _ in range(90):
            if server.poll() is not None:raise RuntimeError(f"Gazebo가 시작 직후 종료됐습니다. {run_dir / 'gazebo.log'} 확인")
            p=subprocess.run(["gz","service","-l"],capture_output=True,text=True,timeout=3)
            if f"/world/{track}/visual_config" in p.stdout:break
            time.sleep(.5)
        else:raise TimeoutError("Gazebo의 신호등 제어 서비스를 찾지 못했습니다.")
        bridge=subprocess.Popen(["ros2","run","ros_gz_bridge","parameter_bridge","/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",*[f"/model/{e['entry_id']}/urrc_timing@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V" for e in entries]],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    except Exception:
        stop(server);log.close()
        raise
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
    # Convert URDF/SDF using the same libsdformat as Gazebo, then add an
    # operator-owned timestamped link-pose publisher to this runtime copy.
    converted = subprocess.run(["gz","sdf","-p",e["_model"]], check=True,
                               capture_output=True,text=True,env=env,timeout=30)
    sdf = ET.fromstring(converted.stdout)
    models = sdf.findall("model")
    if len(models) != 1:
        raise ValueError("제출 차량은 단일 SDF model이어야 합니다.")
    model = models[0]
    base = e.get('base_link_name','base_link')
    if model.find(f"link[@name='{base}']") is None:
        raise ValueError(f"변환된 차량에 실제 {base} 링크가 없습니다. URDF fixed joint 병합 여부를 확인하세요.")
    if model.findtext('static','false').lower() in ('true','1'):
        raise ValueError('참가 차량은 static 모델일 수 없습니다.')
    model.set('name',e['entry_id'])
    plugin = ET.SubElement(model,'plugin',filename='gz-sim-pose-publisher-system',name='gz::sim::systems::PosePublisher')
    for name, value in dict(publish_link_pose='true',publish_model_pose='true',
                           publish_nested_model_pose='true',publish_visual_pose='false',
                           publish_collision_pose='false',publish_sensor_pose='false',
                           use_pose_vector_msg='true',static_publisher='false',
                           update_frequency='200',topic=f"/model/{e['entry_id']}/urrc_timing").items():
        ET.SubElement(plugin,name).text=value
    z=slot["road_z"]+float(e.get("spawn_z_offset_m",.04))
    # Keep relative mesh URIs relative to the original model directory.
    # EntityFactory queues sdf_filename for later processing. Keep the file
    # until the simulator has exited, including delayed mesh/resource loading.
    with tempfile.NamedTemporaryFile(mode='w',suffix='.sdf',prefix='.urrc_',dir=Path(e['_model']).parent,delete=False) as prepared:
        e['_prepared_model']=prepared.name
        prepared.write(ET.tostring(sdf,encoding='unicode'));prepared.flush()
        subprocess.run(["ros2","run","ros_gz_sim","create","--world",track,"--file",prepared.name,"--name",e["entry_id"],"--x",str(slot["x"]),"--y",str(slot["y"]),"--z",str(z),"-Y",str(slot["yaw"])],check=True,env=env,timeout=60)
def remove(track,eid,env):
    gz_service(track,"remove","gz.msgs.Entity",f'name: {json.dumps(eid)} type: MODEL',env=env)
def launch_algo(e,env,run_dir):
    cmd=["ros2","launch",e["ros_package"],e["algorithm_launch_file"],*launch_args(e)]
    path=run_dir/f"{e['entry_id']}_algorithm.log"
    with path.open("a") as log:
        return subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

def validate(root):
    es=load_entries(root)
    import xml.etree.ElementTree as ET
    packages={}
    for e in es:
        for manifest in (Path(e["_folder"])/"src").rglob("package.xml"):
            name=ET.parse(manifest).getroot().findtext("name")
            if name in packages:
                raise ValueError(f"중복 ROS 패키지 이름: {name} ({packages[name]}, {e['entry_id']})")
            if name=="urrc_track_gazebo":raise ValueError("참가물에 운영자 맵 패키지를 포함할 수 없습니다.")
            packages[name]=e["entry_id"]
    for e in es:
        p=subprocess.run(["ros2","pkg","prefix",e["ros_package"]],capture_output=True,text=True)
        if p.returncode:raise ValueError(f"ROS 패키지를 빌드하지 못했습니다: {e['ros_package']}")
        launch=Path(p.stdout.strip())/"share"/e["ros_package"]/"launch"/e["algorithm_launch_file"]
        if not launch.is_file():raise ValueError(f"launch 파일 없음: {launch}")
    import xml.etree.ElementTree as ET
    package=root/"src/urrc_track_gazebo"
    for track in read_json(root/"race/event_config.json")["track_pool"]:
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
    print(f"\n{title}\n순위 | 참가자 | 완료 랩 | 총시간 | 랩별 기록", flush=True)
    for r in rows:
        label = str(r['rank']) + ('(판정 필요)' if r.get('rank_provisional') else '')
        total = duration(r['time_seconds']) if r['status']=='finished' else 'DNF: '+r['reason']
        lap_text = ', '.join(f"{i}랩 {duration(t)}" for i,t in enumerate(r['lap_times_seconds'],1))
        print(f"{label} | {r['display_name']} | {r['laps']} | {total} | {lap_text}", flush=True)


def entry_fingerprints(entries):
    result = {}
    for e in entries:
        h = hashlib.sha256()
        folder = Path(e['_folder'])
        for p in sorted(folder.rglob('*')):
            if not p.is_file() or p.name.startswith('.urrc_') or any(part in {'__pycache__','build','install','log'} for part in p.relative_to(folder).parts):
                continue
            h.update(p.relative_to(folder).as_posix().encode()+b'\0'+p.read_bytes()+b'\0')
        result[e['entry_id']] = h.hexdigest()
    return result


def result_row(e, state, reason='', grid_position=1):
    finished = state.get('finish_time') is not None
    start = state.get('start_time')
    last = state.get('last_lap_time')
    bracket = state.get('finish_bracket')
    return dict(entry_id=e['entry_id'],display_name=e['display_name'],ros_package=e['ros_package'],
                status='finished' if finished else 'DNF',reason='' if finished else reason,
                grid_position=grid_position,laps=state.get('laps',0),
                time_seconds=state['finish_time']-start if finished else None,
                lap_times_seconds=state.get('lap_times_seconds',[]),
                start_sim_time=start,first_crossing_sim_time=state.get('first_crossing'),
                approach_seconds=state.get('approach_seconds'),finish_sim_time=state.get('finish_time'),
                elapsed_bracket_seconds=[t-start for t in bracket] if bracket else None,
                last_lap_elapsed_seconds=last-start if last is not None else None,
                max_sample_gap_seconds=state.get('max_sample_gap_seconds',0.))


def save_results(results, run_dir, mode, payload):
    # Persist partial/invalid results as well. A crash cannot leave an old final
    # looking like the current run. Every invocation keeps its own audit folder.
    for folder in (run_dir,results):
        path = folder/f'{mode}.json'
        temp = path.with_suffix('.json.tmp')
        temp.write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')
        temp.replace(path)
    fields=['rank','rank_provisional','entry_id','display_name','status','reason','laps',
            'time_seconds','total_formatted','lap_times_seconds','grid_position',
            'start_sim_time','finish_sim_time','max_sample_gap_seconds']
    for folder in (run_dir,results):
        with (folder/f'{mode}.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
            for row in payload['results']:
                data={k:row.get(k) for k in fields}
                data['total_formatted']=duration(row.get('time_seconds'))
                data['lap_times_seconds']=json.dumps(row['lap_times_seconds'])
                w.writerow(data)
        with (folder/f'{mode}_laps.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['entry_id','lap','lap_seconds','lap_formatted'])
            for row in payload['results']:
                for i,t in enumerate(row['lap_times_seconds'],1):
                    w.writerow([row['entry_id'],i,t,duration(t)])


def run_mode(root,mode,final_laps=None,start_mode=None):
    cfg=read_json(root/'race/event_config.json')
    start_mode=start_mode or cfg.get('start_mode','signal')
    if start_mode not in ('signal','camera'):
        raise ValueError('출발 방식은 signal/camera만 가능합니다.')
    laps=2 if mode=='qualifying' else (final_laps if final_laps is not None else int(cfg['final_laps']))
    if mode=='qualifying' and final_laps is not None:
        raise ValueError('예선은 2랩 고정입니다. --laps는 본선에서만 사용하세요.')
    if mode=='final' and not 5<=laps<=10:
        raise ValueError('본선 랩 수는 5~10 사이여야 합니다.')
    timeout=float(cfg['lap_timeout_seconds' if mode=='qualifying' else 'race_timeout_seconds'])
    entries=validate(root);event=get_event(root);track=event['track']
    print(f'선택 트랙: {track} / {mode} {laps}랩 / 출발 {start_mode}',flush=True)
    if not entries:
        print('등록된 참가 패키지가 없어 이번 실행은 건너뜁니다. 패키지를 추가한 뒤 다시 실행하세요.')
        return
    for e in entries:e['_start_mode']=start_mode
    fingerprints=entry_fingerprints(entries)
    results=root/'race/results';results.mkdir(exist_ok=True)
    if mode=='final':
        qpath=results/'qualifying.json'
        if not qpath.is_file():raise ValueError('예선을 먼저 완료하세요.')
        q=read_json(qpath)
        if not q.get('valid') or q.get('provisional'):
            raise ValueError('유효한 예선 확정 기록이 필요합니다. 계측 오류/순위 중첩 기록은 재주행하세요.')
        if q['event']['event_id']!=event['event_id'] or q['track']!=track:
            raise ValueError('예선 결과와 이벤트/트랙이 다릅니다.')
        if q.get('entry_fingerprints')!=fingerprints:
            raise ValueError('예선 이후 참가물/설정이 변경됐습니다. 예선을 다시 진행하세요.')
        order={r['entry_id']:i for i,r in enumerate(q['results'])}
        entries.sort(key=lambda e:order[e['entry_id']])
    run_dir=results/'runs'/f"{mode}_{uuid.uuid4().hex[:12]}";run_dir.mkdir(parents=True)
    payload=dict(event=event,track=track,laps=laps,start_mode=start_mode,valid=False,
                 provisional=False,state='running',entry_fingerprints=fingerprints,
                 timing_config={k:cfg[k] for k in ('checkpoint_spacing_m','max_pose_gap_seconds','max_pose_step_m')},
                 audit_directory=str(run_dir.relative_to(root)),results=[])
    save_results(results,run_dir,mode,payload)
    slots=grid(root,track);server=bridge=monitor=gate=log=None
    processes={};spawned=set();out=[];rng=random.Random(event['seed'])

    def check_world():
        if server.poll() is not None or bridge.poll() is not None:
            raise TimingError('Gazebo 또는 중앙 ROS bridge가 종료됐습니다.')
        monitor.check()

    def retire(e):
        eid=e['entry_id']
        monitor.deactivate(eid)
        # Stop the participant controller, then remove the car to clear the line.
        if gate.pubs[eid].get_subscription_count()>0:
            gate.publish([eid],False)
        stop(processes.pop(eid,None))
        if eid in spawned:
            remove(track,eid,env);spawned.remove(eid)

    def record(e, reason, position):
        row=result_row(e,monitor.snapshot(e['entry_id']),reason,position)
        out.append(row)
        print(f"{e['display_name']}: {duration(row['time_seconds']) if row['status']=='finished' else 'DNF '+reason} / {row['laps']}랩",flush=True)
        payload['results']=rank_rows(out)
        save_results(results,run_dir,mode,payload)

    def prepare_entry(e, slot, position):
        eid=e['entry_id']
        try:
            spawn(root,track,e,slot,env);spawned.add(eid)
            if not monitor.wait_seen(eid):
                raise TimingError(f'{eid}: 중앙 base_link 계측을 수신하지 못했습니다.')
            processes[eid]=launch_algo(e,env,run_dir)
            return True
        except (ValueError,ET.ParseError,subprocess.CalledProcessError,subprocess.TimeoutExpired,OSError) as exc:
            check_world()
            record(e,f'차량/알고리즘 준비 실패: {exc}',position)
            retire(e)
            return False

    try:
        server,bridge,env,log=start_world(root,track,event,entries,run_dir)
        monitor=PoseLapMonitor(track,root,entries,run_dir/'timing.jsonl',cfg)
        gate=StartGate(entries)
        groups=[[e] for e in entries] if mode=='qualifying' else [entries]
        for group in groups:
            active=[]
            for e in group:
                position=1 if mode=='qualifying' else entries.index(e)+1
                if prepare_entry(e,slots[position-1],position):active.append(e)
            missing=gate.ready([e['entry_id'] for e in active],processes) if active else []
            for e in list(active):
                if e['entry_id'] in missing:
                    record(e,'출발 구독 미준비 또는 알고리즘 종료',1 if mode=='qualifying' else entries.index(e)+1)
                    retire(e);active.remove(e)
            if not active:continue
            check_world()
            start_sequence(track,root,active,rng.uniform(cfg['lights_out_min_delay_seconds'],cfg['lights_out_max_delay_seconds']),monitor,gate,laps,env)
            deadline=time.monotonic()+timeout
            shown={e['entry_id']:0 for e in active}
            while active:
                check_world()
                monitor.health([e['entry_id'] for e in active])
                completed=[]
                for e in active:
                    eid=e['entry_id'];s=monitor.snapshot(eid)
                    if s['laps']>shown[eid]:
                        for i in range(shown[eid],s['laps']):
                            print(f"{e['display_name']} {i+1}/{laps}랩: {duration(s['lap_times_seconds'][i])}",flush=True)
                        shown[eid]=s['laps']
                    reason=None
                    if s['finish_time'] is not None:reason=''
                    elif processes[eid].poll() is not None:reason='알고리즘 종료'
                    elif time.monotonic()>=deadline:reason='제한시간 초과'
                    if reason is not None:completed.append((e,reason))
                if completed:
                    # Pause the physics while stopping/removing cars, so a slow
                    # process shutdown does not consume another team's race time.
                    pause_begin=time.monotonic()
                    pause_world(track,True,env);monitor.frozen_time()
                    for e,reason in completed:
                        record(e,reason,1 if mode=='qualifying' else entries.index(e)+1)
                        retire(e);active.remove(e)
                    pause_world(track,False,env)
                    monitor.note_resume()
                    deadline+=time.monotonic()-pause_begin
                time.sleep(.01)
        payload.update(valid=True,state='complete',results=rank_rows(out),
                       provisional=any(r.get('rank_provisional') for r in out))
        save_results(results,run_dir,mode,payload)
        print_table(out,'예선 순위' if mode=='qualifying' else '본선 최종 순위')
        print(f"결과/계측 원본: {run_dir}",flush=True)
    except BaseException as exc:
        payload.update(valid=False,state='invalid',error=str(exc),results=rank_rows(out))
        save_results(results,run_dir,mode,payload)
        raise
    finally:
        for proc in processes.values():stop(proc)
        if gate:gate.close()
        if monitor:monitor.close()
        if bridge:stop(bridge)
        if server:stop(server)
        if log:log.close()
        for e in entries:
            if e.get('_prepared_model'):
                Path(e['_prepared_model']).unlink(missing_ok=True)

def main():
    p=argparse.ArgumentParser();p.add_argument("command",choices=("intake","validate","new-event","qualifying","final","status"));p.add_argument("--root",type=Path,default=Path(__file__).resolve().parents[2]);p.add_argument("--track");p.add_argument("--laps",type=int);p.add_argument("--start-mode",choices=("signal","camera"));a=p.parse_args();root=a.root.resolve()
    try:
        if a.command=="intake":print(f"접수 완료: {len(intake(root))}개 참가 패키지")
        elif a.command=="validate":validate(root)
        elif a.command=="new-event":
            results=root/"race/results";results.mkdir(parents=True,exist_ok=True)
            previous=list(results.glob("*.json"))+list(results.glob("*.csv"))
            if previous:
                archive=results/"archive"/uuid.uuid4().hex[:12];archive.mkdir(parents=True)
                for f in previous:f.rename(archive/f.name)
            e=get_event(root,True,a.track)
            print(f"새 이벤트 {e['event_id']} / 트랙 {e['track']}")
        elif a.command in ("qualifying","final"):run_mode(root,a.command,a.laps,a.start_mode)
        else:
            ep=root/"race/event_state.json";print(json.dumps(read_json(ep) if ep.is_file() else {"event":"미생성"},ensure_ascii=False,indent=2))
            for f in (root/"race/results/qualifying.json",root/"race/results/final.json"):
                if f.is_file():print(f"{f.name}: {len(read_json(f)['results'])}개 결과")
    except Exception as exc:print(f"대회 실행 오류: {exc}",file=sys.stderr);return 2
    return 0
if __name__=="__main__":sys.exit(main())
