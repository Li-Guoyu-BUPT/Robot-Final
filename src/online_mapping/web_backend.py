"""Local unified robot console: continuous RGB-D + mapping + one-shot motion.
Run --replay <recorded session> for a motor-disabled UI verification session.
"""
import os
os.environ.setdefault("OMP_NUM_THREADS","8")
import argparse
import collections
import datetime
import json
from pathlib import Path
import secrets
import shlex
import signal
import subprocess
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from urllib.parse import urlparse
import webbrowser
import cv2
import numpy as np

BASE=Path(__file__).resolve().parent.parent
PROJECT=BASE.parent.parent
FRONTEND=PROJECT/"前端"
sys.path.insert(0,str(BASE/"采集"))
sys.path.insert(0,str(BASE/"重建"))
from capture import connect,exact
from map_worker import MapperProxy as LiveMapper
sys.path.insert(0,str(BASE/'遥控'))
from motion_gate import read_motion_interlock

class App:
    def __init__(self,args):
        self.args=args
        self.mode="replay" if args.replay else "live"
        self.motion_interlock=read_motion_interlock()
        self.motion_blocked=self.motion_interlock['blocked']
        stamp=datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session=Path(args.session) if args.session else BASE/("数据派生" if args.replay else "数据")/(("web_replay_" if args.replay else "web_session_")+stamp)
        self.session.mkdir(parents=True,exist_ok=False)
        (self.session/"frames").mkdir()
        self.token=secrets.token_hex(24)
        self.lock=threading.RLock()
        self.send_lock=threading.Lock()
        self.file_lock=threading.Lock()
        self.stop=threading.Event()
        self.mapper=LiveMapper()
        self.latest=None
        self.jpeg=None
        self.last_received=None
        self.received_times=collections.deque(maxlen=80)
        self.received=0
        self.connection="starting"
        self.error=None
        self.events=collections.deque(maxlen=80)
        self.control={"phase":"disarmed","armed":False,"event":"waiting","reason":""}
        self.arm_pending=False
        self.action_pending=False
        self.pending_at=0.
        self.uncertain=False
        self.ui_owner=None
        self.ui_focused=False
        self.ui_seen=0.
        self.seen_commands=collections.OrderedDict()
        self.next_event=0
        self.agent_seq=0
        self.ssh=None
        self.sensor_channel=None
        self.control_channel=None
        self.sensor_pid=None
        self.sensor_script=None
        self.control_thread=None
        self.threads=[]
        self.exports=[]
        self.mapping_enabled=True
        self.map_busy=False
        self.map_generation=0
        self.last_motion=0.
        self.map_ops=threading.RLock()
        self.frames_source=(Path(args.replay)/"frames") if args.replay else self.session/"frames"
        self.archives=[]
        self.optimization={"status":"idle"}
        self.optimization_process=None
        self.scan_archives()
        self.source_done=False
        self.log("session","会话已建立",mode=self.mode,session=str(self.session))

    def log(self,event,text,**data):
        row={"pc_wall":time.time(),"event":event,"text":text}
        row.update(data)
        with self.lock:self.events.append(row)
        with self.file_lock:
            with (self.session/"events.jsonl").open("a",encoding="utf-8") as f:
                f.write(json.dumps(row,ensure_ascii=False)+"\n")

    def start(self):
        if self.mode=="live": self.setup_live()
        else:self.connection="replay"
        targets=[self.receive_live if self.mode=="live" else self.receive_replay,self.mapping_loop]
        if self.mode=="live" and self.control_channel:targets.extend([self.read_control,self.heartbeat_loop])
        for target in targets:
            thread=threading.Thread(target=target,daemon=True)
            self.threads.append(thread);thread.start()

    def setup_live(self):
        self.ssh=connect(self.args.host)
        folder="/tmp/roban_web_"+secrets.token_hex(8)
        sftp=self.ssh.open_sftp()
        sftp.mkdir(folder)
        sftp.put(str(BASE/"采集"/"robot_sensor_agent.py"),folder+"/sensor.py")
        if not self.motion_blocked:
            for name in ("robot_teleop.py","teleop_policy.py","motion_gate.py","motion_interlock.json"):
                sftp.put(str(BASE/"遥控"/name),folder+"/"+name)
        sftp.close()
        self.sensor_script=folder+"/sensor.py"
        env="source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && "
        port=21000+int.from_bytes(secrets.token_bytes(2),"big")%9000
        cmd=env+"echo $$ && exec python -u "+shlex.quote(self.sensor_script)+" --port "+str(port)+" --fps 5"
        _,out,err=self.ssh.exec_command("bash -lc "+shlex.quote(cmd),timeout=20)
        self.sensor_process=out.channel
        self.sensor_pid=int(out.readline().strip())
        if out.readline().strip()!="READY:"+str(port):
            raise RuntimeError("Sensor startup failed: "+err.read().decode(errors="replace"))
        self.sensor_channel=self.ssh.get_transport().open_channel("direct-tcpip",("127.0.0.1",port),("127.0.0.1",0),timeout=8)
        self.sensor_channel.settimeout(5)
        if not self.motion_blocked:
            self.control_channel=self.ssh.get_transport().open_session()
            self.control_channel.exec_command("bash -lc "+shlex.quote(env+"exec python -u "+shlex.quote(folder+"/robot_teleop.py")))
            self.control_channel.settimeout(1)
        else:self.control.update(phase='blocked',armed=False,event='motion_interlock',reason=self.motion_interlock.get('reason'))
        self.connection="connected"
        self.log("connected","机器人已连接；尚未启用运动",host=self.args.host)

    def accept_frame(self,meta,jpg,png,record):
        bgr=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
        depth=cv2.imdecode(np.frombuffer(png,np.uint8),cv2.IMREAD_UNCHANGED)
        if bgr is None or depth is None or depth.dtype!=np.uint16 or bgr.shape[:2]!=depth.shape:
            raise ValueError("RGB-D图像格式错误")
        now=time.monotonic()
        meta=dict(meta)
        meta["pc_received_wall"]=time.time()
        meta["_frames_source"]=str(self.frames_source)
        if record:
            name="%08d"%meta["seq"]
            for ext,data in ((".jpg",jpg),(".png",png),(".json",json.dumps(meta).encode())):
                (self.session/"frames"/(name+ext)).write_bytes(data)
            with (self.session/"frame_manifest.jsonl").open("a",encoding="utf-8") as f:
                f.write(json.dumps(meta)+"\n")
        with self.lock:
            self.received+=1
            self.received_times.append(now)
            self.last_received=now
            self.latest=(self.received,meta,bgr,depth)
            self.jpeg=jpg

    def receive_live(self):
        try:
            while not self.stop.is_set():
                n=struct.unpack("!I",exact(self.sensor_channel,4))[0]
                if not 0<n<=65536:raise ValueError("Invalid frame header")
                meta=json.loads(exact(self.sensor_channel,n))
                nc,nd=int(meta["color_bytes"]),int(meta["depth_bytes"])
                if not (0<nc<2000000 and 0<nd<4000000):raise ValueError("Oversize frame")
                jpg,png=exact(self.sensor_channel,nc),exact(self.sensor_channel,nd)
                self.accept_frame(meta,jpg,png,True)
                self.sensor_channel.sendall(b"1")
        except Exception as exc:
            if not self.stop.is_set():
                self.error=str(exc);self.connection="camera_error"
                self.log("capture_error","采集已中断",error=str(exc))

    def receive_replay(self):
        source=Path(self.args.replay)
        paths=sorted((source/"frames").glob("*.json"))
        try:
            if not paths:raise ValueError("回放目录没有RGB-D帧")
            for p in paths:
                if self.stop.is_set():break
                meta=json.loads(p.read_text(encoding="utf-8-sig"))
                self.accept_frame(meta,p.with_suffix(".jpg").read_bytes(),p.with_suffix(".png").read_bytes(),False)
                self.stop.wait(.2)
            self.source_done=True
            self.log("replay_done","历史数据回放结束，禁止机器人控制")
        except Exception as exc:
            self.error=str(exc);self.log("replay_error","回放失败",error=str(exc))

    def mapping_loop(self):
        last=0
        while not self.stop.is_set():
            with self.lock:frame=self.latest
            if frame is None or frame[0]==last:
                self.stop.wait(.025);continue
            last,meta,bgr,depth=frame
            meta=dict(meta)
            with self.lock:
                moving=self.control.get("phase") in ("pending","executing") or self.arm_pending or self.action_pending
                if moving:self.last_motion=time.monotonic()
                meta["motion_active"]=moving or time.monotonic()-self.last_motion<.55
            try:self.mapper.process(meta,bgr,depth)
            except Exception as exc:
                with self.mapper.lock:
                    self.mapper.state.update(tracking="lost",error=str(exc))
                self.error=str(exc)
                self.log("mapping_error","地图融合暂时冻结",error=str(exc))
                self.stop.wait(.2)

    def send_control(self,message):
        if self.mode!="live" or self.control_channel is None:
            raise RuntimeError("离线回放不允许控制机器人")
        with self.send_lock:
            self.control_channel.sendall((json.dumps(message)+"\n").encode())

    def read_control(self):
        buf=b""
        try:
            while not self.stop.is_set():
                if self.control_channel.recv_ready():
                    buf+=self.control_channel.recv(8192)
                    while b"\n" in buf:
                        line,buf=buf.split(b"\n",1)
                        try:r=json.loads(line)
                        except ValueError:
                            self.log("robot_output",line.decode(errors="replace"));continue
                        with self.lock:
                            self.control.update(r)
                            if r.get("event") in ("armed","stopped","stop_unconfirmed","arm_refused"):
                                self.arm_pending=False;self.action_pending=False
                            if r.get("event") in ("step_request","step_published","step_completed"):
                                self.action_pending=False
                            if r.get("event")=="stop_unconfirmed":self.uncertain=True
                            if r.get('event')=='stopped' and r.get('hardware_stop_confirmed') is True:self.uncertain=False
                        labels={"ready":"单步接口已就绪","armed":"控制已准备","step_published":"单步命令已下发",
                                "step_completed":"步态结束反馈（未验证平衡）","stopped":"已解除启用",
                                "stop_unconfirmed":"本体停止未确认","stop_error":"本体停止请求失败"}
                        self.log("robot",labels.get(r.get("event"),r.get("event","反馈")),feedback=r)
                        if r.get('event') in ('step_completed','stopped','stop_unconfirmed','arm_refused'):
                            threading.Thread(target=self.mapper.release_motion,daemon=True).start()
                if self.control_channel.recv_stderr_ready():
                    self.log("robot_error",self.control_channel.recv_stderr(8192).decode(errors="replace"))
                if self.control_channel.exit_status_ready():
                    raise RuntimeError("机器人单步转发进程已退出")
                self.stop.wait(.02)
        except Exception as exc:
            if not self.stop.is_set():
                with self.lock:self.uncertain=True;self.control.update(armed=False,phase="disarmed")
                self.log("control_error","控制通道已断开",error=str(exc))

    def heartbeat_loop(self):
        while not self.stop.is_set():
            cancel_pending=False
            with self.lock:
                fresh=self.last_received is not None and time.monotonic()-self.last_received<.8
                ui_ok=self.ui_focused and time.monotonic()-self.ui_seen<.7
                executing=self.control.get("phase")=="executing"
                # A dispatched one-shot may finish naturally on a transient video/UI gap.
                # No new step is admitted while the page/camera gate is unhealthy.
                healthy=(fresh and ui_ok) or executing
                self.agent_seq+=1
                seq=self.agent_seq
                if (self.arm_pending or self.action_pending) and time.monotonic()-self.pending_at>6:
                    self.uncertain=True
                    self.arm_pending=False;self.action_pending=False
                    cancel_pending=True
            try:
                if cancel_pending:
                    self.send_control({"op":"stop"})
                    self.log('pending_timeout','准备超时，已取消并请求本体状态核验')
                self.send_control({"op":"heartbeat","seq":seq,"healthy":bool(healthy)})
            except Exception as exc:
                with self.lock:self.uncertain=True
                self.log("heartbeat_error","控制心跳中断",error=str(exc))
                break
            self.stop.wait(.1)

    def front_heartbeat(self,client,focused):
        with self.lock:
            busy=self.control.get("phase") in ("pending","executing") or self.arm_pending or self.action_pending
            inactive=not self.ui_focused or time.monotonic()-self.ui_seen>.7
            take=(self.ui_owner is None or (inactive and not busy and not self.control.get("armed")))
            if focused and take:self.ui_owner=client
            if self.ui_owner==client:
                self.ui_seen=time.monotonic();self.ui_focused=bool(focused)

    def action(self,op,client,command_id,key=None):
        with self.lock:
            dedup=(client,command_id)
            if dedup in self.seen_commands:return self.seen_commands[dedup]
            if self.mode!="live":raise ValueError("历史回放模式禁止机器人控制")
            if op not in ("arm","step","stop"):raise ValueError("未知控制操作")
            if op in ('arm','step') and (self.motion_blocked or read_motion_interlock()['blocked']):
                raise ValueError('步态已停用：现场已报告跌倒，当前只允许相机采集和离线处理')
            if op=='stop' and self.motion_blocked and self.control_channel is None:
                return {'accepted':True,'op':'stop','detail':'sensor_only_no_motion_channel'}
            if op!="stop":
                if self.map_busy:raise ValueError("地图正在清除或恢复，请稍候")
                if self.uncertain:raise ValueError("本体状态无法确认，请停止并检查")
                if self.ui_owner!=client or not self.ui_focused or time.monotonic()-self.ui_seen>.7:
                    raise ValueError("当前页面没有控制焦点")
                if self.last_received is None or time.monotonic()-self.last_received>.8:
                    raise ValueError("相机数据已过期，禁止新增动作")
            if op=="step":
                if key not in ("w","s","a","d","q","e"):raise ValueError("未知方向")
                if self.arm_pending or self.action_pending or self.control.get("phase")!="idle" or not self.control.get("armed"):
                    raise ValueError("尚未启用或当前一步未完成，本次不排队")
                self.next_event+=1
                message={"op":"step","event_id":self.next_event,"key":key}
                self.action_pending=True;self.pending_at=time.monotonic()
            elif op=="arm":
                if self.arm_pending or self.action_pending or self.control.get("phase") in ("pending","executing"):
                    raise ValueError("本体当前忙碌")
                message={"op":"arm"};self.arm_pending=True;self.pending_at=time.monotonic()
            else:message={"op":"stop"}
            result={"accepted":True,"op":op,"event_id":message.get("event_id")}
            self.seen_commands[dedup]=result
            while len(self.seen_commands)>512:self.seen_commands.popitem(last=False)
        transmission_started=False
        try:
            if op=='step':
                self.mapper.prepare_motion()
                with self.lock:
                    unsafe=self.uncertain or not self.ui_focused or time.monotonic()-self.ui_seen>.7 or self.last_received is None or time.monotonic()-self.last_received>.8
                if unsafe:
                    self.mapper.release_motion()
                    raise RuntimeError('准备单步期间页面焦点或实时相机发生变化，本次未下发')
            transmission_started=True
            self.send_control(message)
        except Exception as exc:
            with self.lock:
                self.uncertain=self.uncertain or transmission_started;self.arm_pending=False;self.action_pending=False
                self.seen_commands[dedup]={"accepted":False,"error":"控制发送未确认："+str(exc)}
            raise
        self.log("operator","操作员提交"+op,command=message)
        return result

    def status(self):
        map_state=self.mapper.snapshot()
        with self.lock:
            now=time.monotonic()
            age=None if self.last_received is None else now-self.last_received
            stamps=list(self.received_times)
            fps=(len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 and stamps[-1]>stamps[0] else 0.
            phase="preparing" if self.arm_pending else ("dispatching" if self.action_pending else self.control.get("phase","disarmed"))
            return {"mode":self.mode,"host":self.args.host if self.mode=="live" else "真实历史数据",
                    "connection":self.connection,"session":str(self.session),
                    "capture":{"frames":self.received,"fps":fps,"age_ms":None if age is None else age*1000,
                               "stale":age is None or age>.8,"source_done":self.source_done},
                    "map":map_state,"control":dict(self.control,phase=phase,uncertain=self.uncertain),
                    "events":list(self.events)[-12:],"error":self.error,"exports":list(self.exports),
                    "archives":list(self.archives),"map_busy":self.map_busy,"map_generation":self.map_generation,
                    "optimization":dict(self.optimization),"motion_allowed":not self.motion_blocked,
                    "motion_block_reason":self.motion_interlock.get('reason','')}
    
    def optimization_status(self):
        with self.lock:return dict(self.optimization)

    def scan_archives(self):
        rows=[]
        for path in (BASE/"地图").glob("*/*/checkpoint.json"):
            try:
                row=json.loads(path.read_text(encoding="utf-8"))
                if row.get("algorithm") not in LiveMapper.COMPATIBLE:continue
                rows.append(dict(id=str(path.parent.relative_to(BASE/"地图")),time=row["time"],
                    points=row["points"],session=row["session"],optimized=row["session"].startswith("optimized_")))
            except (OSError,ValueError,KeyError):continue
        self.archives=sorted(rows,key=lambda x:x["time"],reverse=True)[:30]

    def save_map(self):
        with self.map_ops,self.mapper.compute_lock:
            folder=BASE/"地图"/self.session.name/("map_"+datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
            path=self.mapper.save(folder)
            checkpoint={"algorithm":LiveMapper.VERSION,"time":datetime.datetime.now().isoformat(timespec="seconds"),
                "points":json.loads((folder/'map_report.json').read_text(encoding='utf-8'))['points_saved'],"session":self.session.name,"frames_source":str(self.frames_source)}
            (folder/"checkpoint.json").write_text(json.dumps(checkpoint,ensure_ascii=False,indent=2),encoding="utf-8")
            with self.lock:self.exports.append(path);self.scan_archives()
            self.log("map_saved","地图已存档并读回验证",path=path)
            return path

    def change_map(self,archive_id=None):
        with self.map_ops:
            with self.lock:
                if self.arm_pending or self.action_pending or self.control.get("phase") in ("pending","executing"):
                    raise ValueError("请等当前一步完成后再操作地图")
                self.map_busy=True
            try:
                with self.mapper.compute_lock:
                    backup=self.save_map() if self.mapper.snapshot()["integrated"] else None
                    if archive_id is None:
                        self.mapper.reset()
                        self.log("map_cleared","当前地图已清除；旧地图已自动存档，原始数据保留",backup=backup)
                    else:
                        allowed={r["id"] for r in self.archives}
                        if archive_id not in allowed:raise ValueError("存档不在可恢复列表中")
                        folder=(BASE/"地图"/archive_id).resolve()
                        folder.relative_to((BASE/"地图").resolve())
                        checkpoint=json.loads((folder/"checkpoint.json").read_text(encoding="utf-8"))
                        self.mapper.restore(folder,checkpoint["frames_source"])
                        self.log("map_restored","已恢复存档，融合暂时暂停；确认相机位置后继续",archive=archive_id,backup=backup)
                    with self.lock:self.map_generation+=1
                    return {"ok":True,"backup":backup,"paused":self.mapper.snapshot()["paused"]}
            finally:
                with self.lock:self.map_busy=False

    def optimize_map(self):
        with self.lock:
            if self.map_busy or self.optimization.get('status')=='running':raise ValueError('地图操作正在进行')
            if self.arm_pending or self.action_pending or self.control.get('phase') in ('pending','executing'):
                raise ValueError('请等待当前一步完成后优化')
            self.map_busy=True;self.optimization={'status':'running','stage':'archive','current':0,'total':0}
        def work():
            try:
                with self.map_ops:
                    self.mapper.set_paused(True)
                    archive=Path(self.save_map()).parent
                    folder=BASE/'地图'/self.session.name/('optimized_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
                    def progress(**data):
                        with self.lock:self.optimization.update(data)
                    command=[sys.executable,str(BASE/'服务端'/'refine_archive.py'),str(archive),'--out',str(folder)]
                    flags=subprocess.CREATE_NO_WINDOW if os.name=='nt' else 0
                    self.optimization_process=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                        text=True,encoding='utf-8',errors='replace',creationflags=flags)
                    tail=[];failure=None
                    for line in self.optimization_process.stdout:
                        tail.append(line.strip());tail=tail[-6:]
                        try:
                            item=json.loads(line);progress(**item)
                            if item.get('stage')=='failed':failure=item.get('error')
                        except (ValueError,TypeError):pass
                    if self.optimization_process.wait()!=0:
                        self.log('optimization_diagnostics','优化进程诊断',output=tail)
                        raise RuntimeError(failure or '优化进程异常，原图保留；详见本次会话日志')
                    report=json.loads((folder/'map_report.json').read_text(encoding='utf-8'))
                    self.mapper.restore(folder,self.frames_source)
                    with self.lock:
                        self.scan_archives();self.map_generation+=1
                        self.optimization.update(status='done',path=str(folder),
                            old_residual_mm=report['old_median_feature_residual_m']*1000,
                            new_residual_mm=report['optimized_median_feature_residual_m']*1000)
                    self.log('map_optimized','多视角优化已载入；检查后继续融合',path=str(folder))
            except Exception as exc:
                with self.lock:self.optimization.update(status='failed',error=str(exc))
                self.log('optimization_error','优化未通过验证，保留原图及原始数据',error=str(exc))
            finally:
                with self.lock:self.map_busy=False
        thread=threading.Thread(target=work,daemon=True);thread.start()
        return {'accepted':True}

    def close(self):
        if self.mode=="live" and self.control_channel:
            try:
                self.send_control({"op":"stop"})
                self.send_control({"op":"quit"})
                deadline=time.monotonic()+4
                while not self.control_channel.exit_status_ready() and time.monotonic()<deadline:time.sleep(.05)
            except Exception as exc:self.log("cleanup_error","结束控制时需现场确认",error=str(exc))
        self.stop.set()
        if self.optimization_process is not None and self.optimization_process.poll() is None:
            self.optimization_process.terminate()
        if self.sensor_channel:self.sensor_channel.close()
        for thread in self.threads:thread.join(timeout=2)
        if self.ssh:
            try:
                if self.sensor_pid:
                    cmd="if tr '\\0' ' ' < /proc/%d/cmdline 2>/dev/null | grep -Fq -- %s; then kill -TERM %d; fi"%(self.sensor_pid,shlex.quote(self.sensor_script),self.sensor_pid)
                    _,o,e=self.ssh.exec_command(cmd,timeout=5);o.channel.recv_exit_status()
            except Exception as exc:self.log("cleanup_error","采集代理清理需检查",error=str(exc))
            self.ssh.close()
        self.mapper.shutdown()
        self.connection="ended"
        (self.session/"session.json").write_text(json.dumps(self.status(),ensure_ascii=False,indent=2),encoding="utf-8")

class Handler(BaseHTTPRequestHandler):
    protocol_version="HTTP/1.1"
    def log_message(self,*args):pass
    @property
    def app(self):return self.server.app
    def respond(self,data,kind="application/json",code=200,extra=None):
        if isinstance(data,(dict,list)):data=json.dumps(data,ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type",kind)
        self.send_header("Content-Length",str(len(data)))
        self.send_header("Cache-Control","no-store")
        self.send_header("X-Content-Type-Options","nosniff")
        for k,v in (extra or {}).items():self.send_header(k,str(v))
        self.end_headers()
        try:self.wfile.write(data)
        except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):pass
    def do_GET(self):
        route=urlparse(self.path).path
        if route=="/api/status":return self.respond(self.app.status())
        if route=="/api/optimization":return self.respond(self.app.optimization_status())
        if route=="/api/camera.jpg":
            with self.app.lock:data=self.app.jpeg
            return self.respond(data or b"","image/jpeg",200 if data else 204)
        if route=="/api/map.bin":
            with self.app.mapper.lock:data=self.app.mapper.packet;rev=self.app.mapper.state["revision"]
            return self.respond(data,"application/octet-stream",extra={"X-Map-Revision":rev})
        if route=="/api/mesh.bin":
            with self.app.mapper.lock:data=self.app.mapper.mesh_packet;rev=self.app.mapper.state['mesh_revision']
            return self.respond(data,'application/octet-stream',extra={'X-Map-Revision':rev})
        files={"/":("index.html","text/html; charset=utf-8"),"/app.js":("app.js","application/javascript; charset=utf-8"),"/style.css":("style.css","text/css; charset=utf-8")}
        if route not in files:return self.respond({"error":"Not found"},code=404)
        name,kind=files[route]
        data=(FRONTEND/name).read_bytes()
        if name=="index.html":data=data.replace(b"__BOOT__",json.dumps({"token":self.app.token}).encode())
        return self.respond(data,kind)
    def do_POST(self):
        origin=self.headers.get("Origin")
        allowed={"http://127.0.0.1:%s"%self.server.server_port,"http://localhost:%s"%self.server.server_port}
        if self.headers.get("X-Control-Token")!=self.app.token or (origin and origin not in allowed):
            self.close_connection=True
            return self.respond({"error":"请求未授权"},code=403,extra={"Connection":"close"})
        try:
            n=int(self.headers.get("Content-Length","0"))
            if not 0<n<16384:
                self.close_connection=True
                raise ValueError("Invalid payload size")
            data=json.loads(self.rfile.read(n))
            route=urlparse(self.path).path
            if route=="/api/heartbeat":
                client=str(data["client"])
                if not 1<=len(client)<=80:raise ValueError("Invalid client")
                self.app.front_heartbeat(client,bool(data.get("focused")))
                result={"ok":True}
            elif route=="/api/action":
                result=self.app.action(data["op"],str(data["client"]),str(data["command_id"]),data.get("key"))
            elif route=="/api/mapping":
                self.app.mapper.set_paused(bool(data["paused"]));result={"ok":True}
            elif route=="/api/save":
                result={"path":self.app.save_map()}
            elif route=="/api/map/clear":
                result=self.app.change_map()
            elif route=="/api/map/restore":
                result=self.app.change_map(str(data["archive_id"]))
            elif route=="/api/map/optimize":
                result=self.app.optimize_map()
            elif route=="/api/end":
                if self.app.mode=="live" and self.app.control_channel:self.app.send_control({"op":"stop"})
                result={"ending":True}
                self.respond(result)
                threading.Thread(target=self.server.shutdown,daemon=True).start()
                return
            else:return self.respond({"error":"Not found"},code=404)
            self.respond(result)
        except Exception as exc:self.respond({"error":str(exc)},code=409)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--host",default="10.29.201.125")
    parser.add_argument("--port",type=int,default=8765)
    parser.add_argument("--session")
    parser.add_argument("--replay")
    parser.add_argument("--no-open",action="store_true")
    args=parser.parse_args()
    cv2.setNumThreads(2)
    app=App(args)
    server=None
    try:
        app.start()
        server=ThreadingHTTPServer(("127.0.0.1",args.port),Handler)
        server.app=app
        url="http://127.0.0.1:%d"%args.port
        print("ROBAN WEB CONSOLE "+url,flush=True)
        print("SESSION "+str(app.session),flush=True)
        if not args.no_open:webbrowser.open(url)
        server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        if server:server.server_close()
        try:
            if app.mapper.snapshot()["integrated"]:app.save_map()
        except Exception as exc:app.log("save_error","结束时自动保存失败",error=str(exc))
        app.close()
if __name__=="__main__":main()

