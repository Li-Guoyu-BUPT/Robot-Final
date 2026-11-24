"""Controller integration checks with fake ROS; no robot or network."""
import importlib.util
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch
from teleop_policy import Policy

class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.calls=[]
        self.service_created=[]
        self.body='preReady'
        self.owner=0
        self.conflicts=[]
        self.published=[]
        def service(name,kind):
            self.service_created.append(name)
            if name.endswith('GetStatus'): return lambda _:types.SimpleNamespace(data=self.body,poseQueueSize=0,jointQueueSize=0)
            if name.endswith('GetMasterID'): return lambda _:types.SimpleNamespace(data=self.owner)
            def jump(owner,command):
                self.calls.append(command)
                self.owner=owner
                self.body={'setStatus':'ready','walking':'walking','stop':'ready'}.get(command,self.body)
                return types.SimpleNamespace(stateRes=0)
            return jump
        rospy=types.SimpleNamespace(Publisher=lambda *a,**k:types.SimpleNamespace(publish=lambda m:self.published.append(m.data)),
            Subscriber=lambda *a,**k:None,ServiceProxy=service,get_name=lambda:'/test',WARN=1)
        rosgraph=types.SimpleNamespace(Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/gaitCommand',list(self.conflicts))],[],[])))
        msg=types.SimpleNamespace(Bool=object,Float64=object,Float64MultiArray=lambda data:types.SimpleNamespace(data=data))
        srv=types.SimpleNamespace(SrvState=object,SrvString=object,SrvTLSstring=object)
        self.modules=patch.dict(sys.modules,{'rospy':rospy,'rosgraph':rosgraph,'std_msgs':types.ModuleType('std_msgs'),
            'std_msgs.msg':msg,'bodyhub':types.ModuleType('bodyhub'),'bodyhub.srv':srv})
        self.modules.start()
        spec=importlib.util.spec_from_file_location('robot_controller_under_test',Path(__file__).with_name('robot_teleop.py'))
        module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        self.c=module.Controller()
        self.c.motion_gate=lambda:{'blocked':False}
        self.events=[]
        self.c.reply=lambda event,**data:self.events.append((event,data))
    def tearDown(self): self.modules.stop()
    def pending(self):
        import time
        t=time.time()
        self.c.policy.heartbeat(1,t,True)
        self.c.policy.arm(t)
        self.c.policy.submit('w',1,t)
        self.c.hw_ready=True
    def test_many_ros_requests_only_publish_once(self):
        self.pending()
        for _ in range(20): self.c.request(types.SimpleNamespace(data=True))
        self.assertEqual(self.published,[[.05,0.,0.]])
    def test_interlock_never_prepares_servos(self):
        self.c.motion_gate=lambda:{'blocked':True}
        self.c.arm_worker(self.c.generation)
        self.assertEqual(self.calls,[])
        self.assertEqual(self.published,[])
    def test_interlock_blocks_pending_gait(self):
        self.pending();self.c.motion_gate=lambda:{'blocked':True}
        self.c.request(types.SimpleNamespace(data=True))
        self.assertEqual(self.published,[])
    def test_request_before_mode_ready_never_publishes(self):
        self.pending(); self.c.hw_ready=False
        self.c.request(types.SimpleNamespace(data=True))
        self.assertEqual(self.published,[])
    def test_feedback_requires_busy_then_done(self):
        self.pending(); self.c.request(types.SimpleNamespace(data=True))
        self.c.walk_status(types.SimpleNamespace(data=0))
        self.assertEqual(self.c.policy.phase,'executing')
        self.c.walk_status(types.SimpleNamespace(data=1))
        self.c.walk_status(types.SimpleNamespace(data=0))
        self.assertEqual(self.c.policy.phase,'idle')
    def test_conflicting_source_arm_does_not_change_body_state(self):
        import time
        self.conflicts=['/other']; self.c.preparing=True
        self.c.policy.heartbeat(1,time.time(),True)
        self.c.arm_worker(0)
        self.assertEqual(self.calls,[])
        self.assertFalse(self.c.policy.armed)
    def test_f8_prepares_without_entering_walking(self):
        import time
        self.c.preparing=True
        self.c.policy.heartbeat(1,time.time(),True)
        self.c.arm_worker(0)
        self.assertEqual(self.calls,['setStatus'])
        self.assertEqual(self.c.policy.phase,'idle')
        self.assertEqual(self.published,[])
    def test_stopped_generation_never_starts_old_step(self):
        self.pending(); self.owner=88; self.body='ready'
        self.c.stop('operator')
        self.c.prepare_step(0)
        self.assertNotIn('walking',self.calls)
    def test_stop_never_resets_other_owner(self):
        self.pending(); self.owner=2; self.body='walking'
        self.c.stop('operator')
        self.assertEqual(self.calls,[])
    def test_stop_uses_vendor_stop_not_reset_or_extra_gait(self):
        self.pending(); self.owner=88; self.body='walking'
        self.c.stop('operator')
        self.assertEqual(self.calls,['stop'])
        self.assertEqual(self.published,[])


    def test_each_rpc_call_uses_independent_proxy(self):
        before=len(self.service_created)
        self.c.master('a'); self.c.master('b')
        self.assertEqual(len(self.service_created)-before,2)
    def test_idle_focus_loss_does_not_cycle_vendor_walking_mode(self):
        import time
        self.c.policy.heartbeat(1,time.time(),True)
        self.c.preparing=True
        self.c.arm_worker(0)
        self.assertTrue(self.c.owns_control)
        self.body='walking'
        self.c.walk_idle_confirmed=True
        self.calls[:]=[]
        self.c.stop('window_focus_or_camera_lost')
        self.assertEqual(self.calls,[])
        self.c.policy.heartbeat(2,time.time(),True)
        self.c.preparing=True
        self.c.arm_worker(self.c.generation)
        self.assertEqual(self.calls,[])
        self.assertEqual(self.c.policy.phase,'idle')
    def test_idle_unknown_walking_cannot_be_rearmed(self):
        import time
        self.body='walking'; self.owner=88
        self.c.policy.heartbeat(1,time.time(),True)
        self.c.preparing=True
        self.c.arm_worker(0)
        self.assertFalse(self.c.policy.armed)
    def test_stop_is_not_blocked_by_transition_mutex(self):
        self.pending(); self.owner=88; self.body='walking'
        self.c.hardware.acquire()
        try:
            thread=threading.Thread(target=self.c.stop,args=('operator',))
            thread.start()
            thread.join(.5)
            self.assertFalse(thread.is_alive())
        finally:self.c.hardware.release()

    def test_expired_arm_always_reports_refusal(self):
        self.c.preparing=True
        self.c.arm_worker(0)
        self.assertTrue(any(e=='arm_refused' for e,_ in self.events))
        self.assertEqual(self.calls,[])
        self.assertFalse(self.c.preparing)

    def test_stop_checks_body_even_without_ownership_cache(self):
        self.body='ready';self.owner=88
        self.c.stop('operator',force=True)
        self.assertEqual(self.events[-1][0],'stopped')
        self.assertTrue(self.events[-1][1]['hardware_stop_confirmed'])
        self.assertEqual(self.calls,[])

    def test_busy_stop_retries_then_confirms(self):
        self.body='walking';self.owner=88
        original=self.c.jump;attempts=[]
        def jump(owner,command):
            attempts.append(command)
            if len(attempts)==1:raise RuntimeError('FSM service busy')
            return original(owner,command)
        self.c.jump=jump
        self.c.stop('operator',force=True)
        self.assertEqual(attempts,['stop','stop'])
        self.assertTrue(self.events[-1][1]['hardware_stop_confirmed'])
        self.assertEqual(self.published,[])

    def test_persistent_stop_failure_never_unlocks(self):
        self.body='walking';self.owner=88
        def jump(*args):raise RuntimeError('FSM service busy')
        self.c.jump=jump
        self.c.stop('operator',force=True)
        self.assertEqual(self.events[-1][0],'stop_unconfirmed')
        self.assertFalse(self.events[-1][1]['hardware_stop_confirmed'])
        self.assertEqual(self.published,[])

if __name__=='__main__':unittest.main()

