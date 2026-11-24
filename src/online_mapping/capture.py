"""Persistent SSH RGB-D receiver. This program has no motion control path."""
import argparse
import datetime
import getpass
import json
from pathlib import Path
import shlex
import socket
import struct
import threading
import time
import uuid
import cv2
import numpy as np
import paramiko


def exact(channel,n):
    chunks=[]
    while n:
        chunk=channel.recv(n)
        if not chunk: raise EOFError('Sensor stream closed')
        chunks.append(chunk)
        n-=len(chunk)
    return b''.join(chunks)


def connect(host):
    client=paramiko.SSHClient()
    known=Path.home()/'.ssh'/'known_hosts'
    if known.exists(): client.load_host_keys(str(known))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    client.connect(host,username='lemon',password=getpass.getpass('Robot SSH password: '),
                   allow_agent=False,look_for_keys=False,timeout=15,banner_timeout=20,auth_timeout=15)
    client.get_transport().set_keepalive(5)
    return client


def run(host,output,seconds=0,headless=False,fps=10):
    output=Path(output)
    if (output/'frames').exists() and any((output/'frames').glob('*.json')):
        raise FileExistsError('Choose a new session directory; raw recordings are immutable')
    (output/'frames').mkdir(parents=True,exist_ok=True)
    client=connect(host)
    channel=None
    pid=None
    path='/tmp/roban_sensor_'+uuid.uuid4().hex+'.py'
    summary={'host':host,'recording_kind':'synchronized_rgbd_frames','motor_commands_sent':0,
             'world_frame':'initial camera when reconstructed','frames':0,'errors':[]}
    start=time.monotonic()
    samples=[]
    shared={}
    stop=threading.Event()
    def receive():
        try:
            with (output/'frame_manifest.jsonl').open('a',encoding='utf-8') as manifest:
                while not stop.is_set():
                    begin=time.monotonic()
                    n=struct.unpack('!I',exact(channel,4))[0]
                    if not 0<n<=65536: raise ValueError('Invalid packet header')
                    meta=json.loads(exact(channel,n))
                    c,d=int(meta['color_bytes']),int(meta['depth_bytes'])
                    if not (0<c<2000000 and 0<d<4000000): raise ValueError('Oversize frame')
                    jpg,png=exact(channel,c),exact(channel,d)
                    color=cv2.imdecode(np.frombuffer(jpg,np.uint8),cv2.IMREAD_COLOR)
                    depth=cv2.imdecode(np.frombuffer(png,np.uint8),cv2.IMREAD_UNCHANGED)
                    if color is None or depth is None or depth.dtype!=np.uint16 or color.shape[:2]!=depth.shape:
                        raise ValueError('RGB-D decode/type/shape error')
                    if samples and meta['seq']<=samples[-1]['seq']: raise ValueError('Non-monotonic sequence')
                    if samples and meta['color_stamp']<=samples[-1]['color_stamp']: raise ValueError('Non-monotonic capture timestamp')
                    meta['pc_received_wall']=time.time()
                    meta['packet_receive_ms']=(time.monotonic()-begin)*1000
                    name='%08d'%meta['seq']
                    (output/'frames'/(name+'.jpg')).write_bytes(jpg)
                    (output/'frames'/(name+'.png')).write_bytes(png)
                    (output/'frames'/(name+'.json')).write_text(json.dumps(meta),encoding='utf-8')
                    manifest.write(json.dumps(meta)+'\n')
                    manifest.flush()
                    samples.append(meta)
                    shared['latest']=(color,depth,time.monotonic())
                    channel.sendall(b'1')
        except Exception as exc:
            if not stop.is_set(): summary['errors'].append(repr(exc))
            stop.set()
    receiver=None
    remote=None
    port=19000+int(uuid.uuid4().hex[:4],16)%10000
    try:
        sftp=client.open_sftp()
        sftp.put(str(Path(__file__).with_name('robot_sensor_agent.py')),path)
        sftp.close()
        env='source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && '
        command=env+'echo $$ && exec python -u '+shlex.quote(path)+' --fps '+str(float(fps))+' --port '+str(port)
        _,stdout,stderr=client.exec_command('bash -lc '+shlex.quote(command),timeout=20)
        remote=stdout.channel
        pid=int(stdout.readline().strip())
        ready=stdout.readline().strip()
        if ready!='READY:'+str(port):
            raise RuntimeError('Sensor initialization failed: '+stderr.read().decode(errors='replace'))
        for _ in range(25):
            try:
                channel=client.get_transport().open_channel('direct-tcpip',('127.0.0.1',port),('127.0.0.1',0),timeout=5)
                break
            except paramiko.ChannelException:
                if remote.exit_status_ready(): raise RuntimeError(stderr.read().decode(errors='replace'))
                time.sleep(.3)
        if channel is None: raise RuntimeError('Sensor stream unavailable')
        channel.settimeout(10)
        receiver=threading.Thread(target=receive,daemon=True)
        receiver.start()
        start=time.monotonic()
        while not stop.is_set() and (not seconds or time.monotonic()-start<seconds):
            frame=shared.get('latest')
            if frame and not headless:
                color,depth,received=frame
                preview=color.copy()
                cv2.putText(preview,'RECORDING %d frames | Q/ESC finish'%len(samples),(10,25),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0),1)
                cv2.imshow('ROBAN RGB-D recording (no motor control)',preview)
                if cv2.waitKey(20)&255 in (ord('q'),27): break
            else: time.sleep(.05)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        summary['errors'].append(repr(exc))
    finally:
        stop.set()
        if channel: channel.close()
        if receiver: receiver.join(timeout=12)
        if pid:
            # Check that this PID still executes our uniquely named sensor script.
            cleanup='if tr "\\0" " " < /proc/%d/cmdline 2>/dev/null | grep -Fq -- %s; then kill -TERM %d; fi'%(pid,shlex.quote(path),pid)
            try:
                _,s,e=client.exec_command(cleanup,timeout=5)
                s.channel.recv_exit_status()
                summary['agent_cleanup_requested']=True
            except Exception as exc: summary['cleanup_error']=repr(exc)
        if remote:
            messages=[]
            while remote.recv_stderr_ready():
                messages.append(remote.recv_stderr(65536).decode(errors='replace'))
            (output/'robot_agent_stderr.log').write_text(''.join(messages),encoding='utf-8')
        client.close()
        if not headless: cv2.destroyAllWindows()
        summary['frames']=len(samples)
        summary['elapsed_s']=time.monotonic()-start
        if samples:
            span=samples[-1]['color_stamp']-samples[0]['color_stamp']
            pc_span=samples[-1]['pc_received_wall']-samples[0]['pc_received_wall']
            robot_span=samples[-1]['robot_encoded_wall']-samples[0]['robot_encoded_wall']
            summary.update(capture_span_s=span,effective_fps=(len(samples)-1)/span if span>0 else 0,
                           pc_receive_span_s=pc_span,pc_receive_fps=(len(samples)-1)/pc_span if pc_span>0 else 0,
                           queue_growth_estimate_s=pc_span-robot_span,
                           mean_packet_bytes=sum(s['color_bytes']+s['depth_bytes'] for s in samples)/len(samples),
                           max_pair_delta_ms=max(s['pair_delta_ms'] for s in samples),
                           mean_encode_ms=sum(s['encode_ms'] for s in samples)/len(samples),
                           skipped_sensor_frames=samples[-1]['seq']-samples[0]['seq']+1-len(samples))
        (output/'session.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--host',default='10.29.201.125')
    parser.add_argument('--out',default=str(Path(__file__).resolve().parent.parent/'数据'/('session_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))))
    parser.add_argument('--seconds',type=float,default=0)
    parser.add_argument('--fps',type=float,default=10)
    parser.add_argument('--headless',action='store_true')
    args=parser.parse_args()
    result=run(args.host,args.out,args.seconds,args.headless,args.fps)
    print(json.dumps(result,indent=2))
    if result['errors'] or not result['frames']: raise SystemExit(1)
