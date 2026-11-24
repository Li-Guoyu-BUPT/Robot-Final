#!/usr/bin/env python
from __future__ import print_function
import json
import sys
import threading
import time
import rospy
import rosgraph
from std_msgs.msg import Bool,Float64,Float64MultiArray
from bodyhub.srv import SrvState,SrvString,SrvTLSstring
from teleop_policy import Policy,COMMANDS
from motion_gate import read_motion_interlock

class Controller(object):
    def __init__(self):
        self.policy=Policy()
        self.owner=88
        self.motion_gate=read_motion_interlock
        self.lock=threading.RLock()
        self.hardware=threading.RLock()
        self.output_lock=threading.Lock()
        self.done=False
        self.preparing=False
        self.stopping=False
        self.hw_ready=False
        self.owns_control=False
        self.walk_idle_confirmed=False
        self.generation=0
        self.pub=rospy.Publisher('/gaitCommand',Float64MultiArray,queue_size=1)
        self.state=lambda text: rospy.ServiceProxy('/MediumSize/BodyHub/GetStatus',SrvString)(text)
        self.master=lambda text: rospy.ServiceProxy('/MediumSize/BodyHub/GetMasterID',SrvTLSstring)(text)
        self.jump=lambda owner,command: rospy.ServiceProxy('/MediumSize/BodyHub/StateJump',SrvState)(owner,command)
        self.sub=rospy.Subscriber('/requestGaitCommand',Bool,self.request,queue_size=1)
        self.walk_sub=rospy.Subscriber('/MediumSize/BodyHub/WalkingStatus',Float64,self.walk_status,queue_size=4)

    def reply(self,event,**data):
        with self.lock:
            data.update(event=event,phase=self.policy.phase,armed=self.policy.armed,
                        event_id=self.policy.event_id,robot_wall=time.time(),stopping=self.stopping)
        with self.output_lock:
            print(json.dumps(data))
            sys.stdout.flush()

    def publishers(self):
        pubs=rosgraph.Master(rospy.get_name()).getSystemState()[0]
        return [n for topic,nodes in pubs if topic=='/gaitCommand' for n in nodes if n!=rospy.get_name()]

    def stop(self,reason,force=False):
        with self.lock:
            old_phase=self.policy.phase
            needs_vendor_stop=(old_phase in ('pending','executing') or self.preparing or force)
            self.generation+=1
            self.policy.cancel()
            self.preparing=False
            self.hw_ready=False
        confirmed=None
        if needs_vendor_stop:
            self.stopping=True
            self.walk_idle_confirmed=False
            # Cancellation is immediate. A stop must not race a StateJump in
            # preparation, nor block the command reader waiting for that call.
            if not self.hardware.acquire(False):
                self.reply('stopping',reason=reason)
                def deferred():
                    deadline=time.time()+3.
                    while time.time()<deadline:
                        if self.hardware.acquire(False):
                            try:self.finish_stop(reason)
                            finally:self.hardware.release()
                            return
                        time.sleep(.05)
                    self.reply('stop_unconfirmed',reason=reason,hardware_stop_confirmed=False,
                               scope='vendor',error='Preparation service still busy; press stop to check again')
                thread=threading.Thread(target=deferred);thread.daemon=True;thread.start()
                return
            try:self.finish_stop(reason)
            finally:self.hardware.release()
            return
        self.reply('stop_unconfirmed' if confirmed is False else 'stopped',reason=reason,
                   scope='vendor' if needs_vendor_stop else 'software_idle',
                   hardware_stop_confirmed=confirmed)

    def finish_stop(self,reason):
        error=None
        for attempt in range(3):
            try:
                owner=self.master('single_step').data
                state=self.state('single_step')
                if owner not in (0,self.owner) or self.publishers():
                    raise RuntimeError('Control owner or publisher conflict')
                if state.data=='walking' and owner==self.owner:
                    self.jump(self.owner,'stop')
                    state=self.state('single_step')
                confirmed=(state.data in ('ready','pause','preReady') and
                           not state.poseQueueSize and not state.jointQueueSize and
                           self.master('single_step').data in (0,self.owner))
                if confirmed:
                    self.stopping=False
                    self.reply('stopped',reason=reason,scope='vendor',hardware_stop_confirmed=True)
                    return
                error='BodyHub not settled: '+state.data
            except Exception as exc:error=str(exc)
            if attempt<2:time.sleep(.15)
        self.reply('stop_unconfirmed',reason=reason,scope='vendor',
                   hardware_stop_confirmed=False,error=error)

    def arm_worker(self,token):
        try:
            if self.motion_gate()['blocked']:raise RuntimeError('Motion interlock is active')
            with self.hardware:
                conflicts=self.publishers()
                s=self.state('single_step')
                owner=self.master('single_step').data
                if conflicts: raise RuntimeError('Other gait publishers: '+','.join(conflicts))
                reuse_idle=(s.data=='walking' and owner==self.owner and self.owns_control and self.walk_idle_confirmed)
                if (s.data not in ('preReady','ready','pause') and not reuse_idle) or owner not in (0,self.owner) or s.poseQueueSize or s.jointQueueSize:
                    raise RuntimeError('BodyHub not available: '+s.data+', owner='+str(owner))
                with self.lock:
                    if token!=self.generation or not self.policy.fresh(time.time()):
                        self.reply('arm_refused',reason='Preparation cancelled or page heartbeat expired')
                        return
                # Only the operator F8 initiates preparation. No reset or walking here.
                if not reuse_idle: self.jump(self.owner,'setStatus')
                if self.state('single_step').data not in (('walking',) if reuse_idle else ('ready','pause')) or self.master('single_step').data!=self.owner:
                    raise RuntimeError('Preparation/ownership not confirmed')
                with self.lock:
                    if token!=self.generation:
                        self.reply('arm_refused',reason='Preparation cancelled')
                        return
                    self.owns_control=True
                    if not reuse_idle: self.walk_idle_confirmed=False
                    if not self.policy.arm(time.time()): raise RuntimeError('Heartbeat expired during preparation')
                self.reply('armed',note='Tap one direction; SPACE stops. F8 preparation may initialize servos.')
        except Exception as exc:
            self.stop('arm_refused: '+str(exc))
        finally:
            with self.lock:
                if token==self.generation: self.preparing=False

    def prepare_step(self,token):
        try:
            with self.hardware:
                if self.publishers() or self.master('single_step').data!=self.owner:
                    raise RuntimeError('Control source conflict')
                with self.lock:
                    if token!=self.generation or self.policy.phase!='pending': return
                s=self.state('single_step')
                if s.data in ('ready','pause'):
                    self.jump(self.owner,'walking')
                if self.state('single_step').data!='walking': raise RuntimeError('Walking mode unavailable')
                with self.lock:
                    if token==self.generation and self.policy.phase=='pending': self.hw_ready=True
        except Exception as exc:
            self.stop('step_prepare_failed: '+str(exc))

    def request(self,msg):
        if not msg.data: return
        if self.motion_gate()['blocked']:
            self.stop('motion_interlock',force=True)
            return
        with self.lock:
            if not self.hw_ready: return
            cmd=self.policy.take(time.time())
            if cmd is None: return
            # The pending command is consumed before publication. Repeated requests do not repeat it.
            self.pub.publish(Float64MultiArray(data=list(cmd)))
            self.hw_ready=False
        self.reply('step_published',command=list(cmd))

    def walk_status(self,msg):
        with self.lock:
            before=self.policy.phase
            unexpected=(before=='idle' and self.policy.armed and msg.data>0)
            if msg.data>0: self.walk_idle_confirmed=False
            complete=self.policy.observe(msg.data,time.time())
            if complete: self.walk_idle_confirmed=True
        if before=='executing': self.reply('walking_feedback',value=msg.data)
        if complete: self.reply('step_completed')
        if unexpected: self.stop('unexpected_walking_feedback',force=True)

    def read_commands(self):
        try:
            while not self.done:
                line=sys.stdin.readline(4097)
                if not line: break
                if len(line)>4096: raise ValueError('Oversize command')
                message=json.loads(line)
                op=message.get('op')
                if op=='heartbeat':
                    with self.lock:
                        accepted=self.policy.heartbeat(int(message['seq']),time.time(),message.get('healthy',False))
                        unsafe=accepted and self.policy.armed and not self.policy.healthy
                    if unsafe: self.stop('window_focus_or_camera_lost')
                elif op=='arm':
                    if self.motion_gate()['blocked']:
                        self.reply('arm_refused',reason='motion_interlock_active')
                        continue
                    with self.lock:
                        if self.stopping or self.preparing or self.policy.phase!='disarmed':
                            self.reply('arm_refused',reason='already armed or busy')
                            continue
                        self.preparing=True
                        token=self.generation
                    threading.Thread(target=self.arm_worker,args=(token,)).start()
                elif op=='step':
                    if self.motion_gate()['blocked']:
                        self.reply('step_request',result='motion_interlock_active')
                        continue
                    with self.lock:
                        result=self.policy.submit(message.get('key'),int(message['event_id']),time.time())
                        token=self.generation
                        if result=='accepted': self.walk_idle_confirmed=False
                    self.reply('step_request',result=result)
                    if result=='accepted':
                        threading.Thread(target=self.prepare_step,args=(token,)).start()
                elif op=='stop': self.stop('operator',force=True)
                elif op=='quit': break
        except Exception as exc:
            self.reply('protocol_error',error=str(exc))
        finally:
            self.stop('command_channel_closed',force=True)
            self.done=True

    def run(self):
        thread=threading.Thread(target=self.read_commands)
        thread.daemon=True
        thread.start()
        self.reply('ready',conflicts=self.publishers(),mode='one_gait_command_per_press',command_steps=COMMANDS)
        last_audit=0
        while not self.done and not rospy.is_shutdown():
            with self.lock:
                fault=self.policy.fault(time.time())
            if fault:
                self.stop(fault)
            elif self.policy.armed and time.time()-last_audit>.5:
                last_audit=time.time()
                try:
                    if self.publishers() or self.master('single_step').data!=self.owner:
                        self.stop('control_source_conflict')
                except Exception as exc: self.stop('control_audit_failed: '+str(exc))
            time.sleep(.025)

if __name__=='__main__':
    if read_motion_interlock()['blocked']:
        print(json.dumps({'event':'motion_interlock','armed':False,'reason':'Fall investigation; motion disabled'}))
        sys.exit(2)
    rospy.init_node('roban_manual_single_step',anonymous=True,log_level=rospy.WARN)
    Controller().run()


