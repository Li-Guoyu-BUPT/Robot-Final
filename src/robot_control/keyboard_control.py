"""Manual single-step keyboard. Dry-run by default; no automatic gait."""
import argparse
import ctypes
import datetime
import json
from pathlib import Path
import shlex
import sys
import threading
import time
import uuid
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'采集'))
from capture import connect
from teleop_policy import COMMANDS,KeyEdges,Policy
from motion_gate import read_motion_interlock

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',default='10.29.201.125')
    parser.add_argument('--live',action='store_true')
    parser.add_argument('--session')
    parser.add_argument('--smoke-test',action='store_true')
    parser.add_argument('--simulate-render-stall',action='store_true')
    args=parser.parse_args()
    if args.live and read_motion_interlock()['blocked']:
        parser.error('步态测试已停用：现场已报告跌倒。只允许离线演练或传感器采集。')
    if args.simulate_render_stall and not args.smoke_test: parser.error('Render-stall check is smoke-test only')
    if args.live and (not args.session or args.smoke_test): parser.error('Live requires --session; smoke-test is dry-run only')
    title='ROBAN SINGLE STEP LIVE' if args.live else 'ROBAN SINGLE STEP DRY RUN'
    log_dir=Path(args.session) if args.session else Path(__file__).resolve().parent/'演练记录'
    log_dir.mkdir(parents=True,exist_ok=True)
    log_path=log_dir/('keyboard_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'.jsonl')
    log=log_path.open('a',encoding='utf-8')
    log_lock=threading.Lock()
    state_lock=threading.Lock()
    state={'phase':'disarmed','event':'starting','result':''}
    client=None
    channel=None
    simulator=Policy()
    sim_finish=None
    def write(role,message):
        with log_lock:
            log.write(json.dumps({'pc_wall':time.time(),'role':role,'message':message},ensure_ascii=False)+'\n')
            log.flush()
    def feedback(message):
        write('robot' if args.live else 'simulation',message)
        with state_lock:
            if 'phase' in message: state['phase']=message['phase']
            state['event']=message.get('event','feedback')
            state['result']=str(message.get('reason',message.get('result','')))
        print(json.dumps(message,ensure_ascii=False),flush=True)
    def simulated_feedback(event,**data):
        data.update(event=event,phase=simulator.phase,armed=simulator.armed)
        feedback(data)
    def send(message):
        nonlocal sim_finish
        write('operator',message)
        if channel:
            channel.sendall((json.dumps(message)+'\n').encode())
        else:
            now=time.monotonic()
            op=message['op']
            if op=='heartbeat':
                simulator.heartbeat(message['seq'],now,message['healthy'])
                if simulator.armed and not message['healthy']:
                    simulator.cancel(); sim_finish=None
                    simulated_feedback('stopped',reason='focus_or_camera')
            elif op=='arm':
                ok=simulator.arm(now)
                simulated_feedback('armed' if ok else 'arm_refused')
            elif op=='step':
                result=simulator.submit(message['key'],message['event_id'],now)
                simulated_feedback('step_request',result=result)
                if result=='accepted':
                    cmd=simulator.take(now)
                    simulator.observe(1,now)
                    sim_finish=now+1.
                    simulated_feedback('simulated_step',command=cmd)
            elif op in ('stop','quit'):
                simulator.cancel(); sim_finish=None
                simulated_feedback('stopped')

    try:
        if args.live:
            client=connect(args.host)
            folder='/tmp/roban_single_keys_'+uuid.uuid4().hex
            sftp=client.open_sftp()
            sftp.mkdir(folder)
            for name in ('robot_teleop.py','teleop_policy.py','motion_gate.py','motion_interlock.json'):
                sftp.put(str(Path(__file__).with_name(name)),folder+'/'+name)
            sftp.close()
            command='source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && exec python -u '+shlex.quote(folder+'/robot_teleop.py')
            channel=client.get_transport().open_session()
            channel.exec_command('bash -lc '+shlex.quote(command))
            channel.settimeout(1)
            def read():
                buf=b''
                while not channel.closed:
                    if channel.recv_ready():
                        buf+=channel.recv(8192)
                        while b'\n' in buf:
                            line,buf=buf.split(b'\n',1)
                            try: feedback(json.loads(line.decode()))
                            except (ValueError,UnicodeError): write('robot_stdout',line.decode(errors='replace'))
                    if channel.recv_stderr_ready():
                        write('robot_stderr',channel.recv_stderr(8192).decode(errors='replace'))
                    if channel.exit_status_ready(): break
                    time.sleep(.02)
            reader=threading.Thread(target=read,daemon=True)
            reader.start()
        edges=KeyEdges()
        input_stop=threading.Event()
        input_errors=[]
        cv2.namedWindow(title)
        cv2.imshow(title,np.zeros((285,850,3),np.uint8))
        cv2.waitKey(1)
        window_started=time.monotonic()
        def input_worker():
            nonlocal sim_finish
            previous_arm=False
            previous_stop=False
            sequence=0
            event_id=0
            user32=ctypes.windll.user32
            user32.FindWindowW.restype=ctypes.c_void_p
            user32.GetForegroundWindow.restype=ctypes.c_void_p
            try:
                while not input_stop.is_set():
                    if not args.live and sim_finish is not None and time.monotonic()>=sim_finish:
                        simulator.observe(0,time.monotonic())
                        sim_finish=None
                        simulated_feedback('step_completed')
                    if channel and (channel.closed or channel.exit_status_ready()):
                        raise ConnectionError('Remote controller exited; see log')
                    focused=user32.GetForegroundWindow()==user32.FindWindowW(None,title)
                    pressed=lambda code:bool(user32.GetAsyncKeyState(code)&0x8000)
                    camera_fresh=True
                    if args.live:
                        manifest=Path(args.session)/'frame_manifest.jsonl'
                        camera_fresh=manifest.is_file() and 0<=time.time()-manifest.stat().st_mtime<.8
                    sequence+=1
                    send({'op':'heartbeat','seq':sequence,'healthy':bool(focused and camera_fresh)})
                    if focused and pressed(0x1B):
                        send({'op':'stop'})
                        input_stop.set()
                        break
                    arm=focused and pressed(0x77)
                    stop_key=focused and pressed(0x20)
                    keys=[k for k in COMMANDS if pressed(ord(k.upper()))]
                    key=edges.poll(keys,focused)
                    if stop_key and not previous_stop:
                        send({'op':'stop'})
                    elif arm and not previous_arm:
                        if not keys and camera_fresh: send({'op':'arm'})
                        else: feedback({'event':'arm_refused','reason':'Release direction keys and check camera'})
                    elif key and not stop_key and not arm:
                        event_id+=1
                        if camera_fresh: send({'op':'step','event_id':event_id,'key':key})
                        else: feedback({'event':'step_refused','reason':'Camera recording is stale'})
                    previous_arm=arm
                    previous_stop=stop_key
                    input_stop.wait(.045)
            except Exception as exc:
                input_errors.append(repr(exc))
                input_stop.set()
        worker=threading.Thread(target=input_worker,daemon=True)
        worker.start()
        simulated_stall=False
        try:
            while not input_stop.is_set():
                if args.smoke_test and time.monotonic()-window_started>2: break
                if args.simulate_render_stall and not simulated_stall and time.monotonic()-window_started>.25:
                    simulated_stall=True
                    time.sleep(1.5)
                with state_lock: status=dict(state)
                canvas=np.zeros((285,850,3),np.uint8)
                lines=[title,'F8: prepare/arm (servo initialization possible; no gait yet)',
                       'Tap W/S forward/back; A/D lateral; Q/E turn ONCE.',
                       'Holding a direction does NOT repeat. Release before next press.',
                       'SPACE: controlled stop + disarm. ESC: stop and close.',
                       'Phase: '+status['phase']+' | '+status['event'],status['result'][:95]]
                for i,text in enumerate(lines):
                    cv2.putText(canvas,text,(12,30+34*i),cv2.FONT_HERSHEY_SIMPLEX,.53,(210,230,220),1)
                cv2.imshow(title,canvas)
                if cv2.waitKey(15)&255==27: break
                if cv2.getWindowProperty(title,cv2.WND_PROP_VISIBLE)<1: break
            if input_errors: raise RuntimeError(input_errors[-1])
        finally:
            input_stop.set()
            worker.join(timeout=2)
    except KeyboardInterrupt:
        pass
    finally:
        if channel:
            try:
                send({'op':'stop'})
                send({'op':'quit'})
                limit=time.monotonic()+5
                while time.monotonic()<limit and not channel.exit_status_ready(): time.sleep(.05)
            except Exception: pass
            channel.close()
        if client: client.close()
        if not args.live: send({'op':'quit'})
        log.close()
        cv2.destroyAllWindows()

if __name__=='__main__': main()

