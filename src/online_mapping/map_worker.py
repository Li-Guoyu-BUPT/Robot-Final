"""Isolate legacy Open3D's GIL-holding work from camera and control heartbeats."""
import multiprocessing as mp
import queue,threading,time,uuid
from pathlib import Path

def latest_put(channel,value):
    try:channel.put_nowait(value);return
    except queue.Full:pass
    try:channel.get_nowait()
    except queue.Empty:pass
    try:channel.put_nowait(value)
    except queue.Full:pass

def worker_main(frames,commands,states,geometry,results):
    import cv2,numpy as np
    from live_mapper_v3 import LiveMapper
    cv2.setNumThreads(2)
    mapper=LiveMapper();last_revision=-1;done=False;motion_hold=False
    def publish(force=False):
        nonlocal last_revision
        state=mapper.snapshot()
        latest_put(states,state)
        if force or state['revision']!=last_revision:
            latest_put(geometry,(state,mapper.packet,mapper.mesh_packet))
            last_revision=state['revision']
    publish(True)
    while not done:
        try:command=commands.get_nowait()
        except queue.Empty:command=None
        if command:
            cid,op,args=command
            try:
                if op=='quit':done=True;value=True
                elif op=='save':value=mapper.save(*args)
                elif op=='restore':mapper.restore(*args);value=True
                elif op=='reset':mapper.reset();value=True
                elif op=='pause':mapper.set_paused(*args);value=True
                elif op=='motion':
                    motion_hold=bool(args[0]);value=True
                    mapper.motion(motion_hold)
                else:raise ValueError('Unknown mapper operation')
                publish(op in ('restore','reset'))
                results.put((cid,True,value,mapper.snapshot()))
            except Exception as exc:results.put((cid,False,str(exc),None))
            continue
        try:meta=frames.get(timeout=.05)
        except queue.Empty:continue
        # Drain to the latest disk-backed frame, never build a processing backlog.
        while True:
            try:meta=frames.get_nowait()
            except queue.Empty:break
        try:
            if motion_hold:meta['motion_active']=True
            stem=Path(meta['_frames_source'])/('%08d'%meta['seq'])
            color=cv2.imdecode(np.frombuffer(stem.with_suffix('.jpg').read_bytes(),np.uint8),1)
            depth=cv2.imdecode(np.frombuffer(stem.with_suffix('.png').read_bytes(),np.uint8),-1)
            mapper.process(meta,color,depth)
        except Exception as exc:
            with mapper.lock:mapper.state.update(tracking='lost',error=str(exc),fusion_blocked='重建异常，保留已有地图')
        publish()

class MapperProxy:
    VERSION='model_guarded_v3'
    COMPATIBLE=(VERSION,'keyframe_filtered_v2','pose_graph_refined_v3')
    def __init__(self):
        self.lock=threading.RLock();self.compute_lock=threading.RLock();self.start_lock=threading.Lock()
        self.state=dict(tracking='waiting',processed=0,integrated=0,failed=0,revision=0,
            points=0,display_points=0,processing_ms=0.,error=None,paused=False,trajectory=[],mesh_revision=0,triangles=0)
        self.packet=b'';self.mesh_packet=b'';self.child=None;self.pending={};self.stopped=threading.Event()
        self.geometry_revision=-1

    def start(self):
        with self.start_lock:
            if self.child is not None:return
            context=mp.get_context('spawn')
            self.frames=context.Queue(maxsize=2);self.commands=context.Queue(maxsize=8)
            self.states=context.Queue(maxsize=4);self.geometry=context.Queue(maxsize=2);self.results=context.Queue()
            self.child=context.Process(target=worker_main,args=(self.frames,self.commands,self.states,self.geometry,self.results),daemon=True)
            self.child.start()
            for target in (self.read_states,self.read_geometry,self.read_results):
                threading.Thread(target=target,daemon=True).start()

    def read_states(self):
        while not self.stopped.is_set():
            try:state=self.states.get(timeout=.2)
            except queue.Empty:continue
            with self.lock:
                # Publish only geometry revisions whose actual buffers have arrived.
                visual={k:self.state.get(k,0) for k in ('revision','mesh_revision','points','display_points','triangles')}
                self.state.update(state);self.state.update(visual)

    def read_geometry(self):
        while not self.stopped.is_set():
            try:state,packet,mesh=self.geometry.get(timeout=.2)
            except queue.Empty:continue
            with self.lock:
                if state['revision']<self.geometry_revision:continue
                self.geometry_revision=state['revision']
                self.packet=packet;self.mesh_packet=mesh
                for key in ('revision','mesh_revision','points','display_points','triangles'):self.state[key]=state[key]

    def read_results(self):
        while not self.stopped.is_set():
            try:cid,ok,value,state=self.results.get(timeout=.2)
            except queue.Empty:continue
            with self.lock:pending=self.pending.get(cid)
            if pending is not None:
                pending['result']=(ok,value,state);pending['event'].set()

    def rpc(self,op,*args):
        self.start();cid=uuid.uuid4().hex;pending={'event':threading.Event()}
        with self.lock:self.pending[cid]=pending
        self.commands.put((cid,op,args),timeout=2)
        try:
            deadline=time.monotonic()+120
            while not pending['event'].wait(.2):
                if not self.child.is_alive():raise RuntimeError('重建进程已退出，原始相机采集继续')
                if time.monotonic()>deadline:raise TimeoutError('重建操作超时，保留原始数据')
            ok,value,state=pending['result']
            if not ok:raise RuntimeError(value)
            if op in ('restore','reset'):
                while self.geometry_revision<state['revision']:
                    if time.monotonic()>deadline:raise TimeoutError('地图已计算，显示缓冲同步超时')
                    self.stopped.wait(.01)
            if state is not None:
                with self.lock:
                    visual={k:self.state.get(k,0) for k in ('revision','mesh_revision','points','display_points','triangles')}
                    self.state.update(state);self.state.update(visual)
            return value
        finally:
            with self.lock:self.pending.pop(cid,None)

    def process(self,meta,bgr,depth):
        self.start();latest_put(self.frames,dict(meta))

    def snapshot(self):
        with self.lock:return dict(self.state)

    def set_paused(self,value):
        result=self.rpc('pause',bool(value))
        with self.lock:self.state['paused']=bool(value)
        return result

    def save(self,folder):return self.rpc('save',str(folder))
    def restore(self,folder,frames_source):return self.rpc('restore',str(folder),str(frames_source))
    def reset(self):return self.rpc('reset')
    def prepare_motion(self):return self.rpc('motion',True)
    def release_motion(self):return self.rpc('motion',False)

    def shutdown(self):
        if self.child is None:return
        try:self.rpc('quit')
        except Exception:pass
        self.stopped.set();self.child.join(timeout=3)
        if self.child.is_alive():self.child.terminate();self.child.join(timeout=2)
