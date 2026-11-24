"""Admission tests with a fake transport. Never connects to a robot."""
import concurrent.futures
from pathlib import Path
import tempfile
import time
import types
import unittest
from web_backend import App
from unittest.mock import patch


class AdmissionTests(unittest.TestCase):
    def setUp(self):
        self.gate_patch=patch('web_backend.read_motion_interlock',return_value={'blocked':False})
        self.gate_patch.start()
        self.temp=tempfile.TemporaryDirectory()
        args=types.SimpleNamespace(replay=None,session=str(Path(self.temp.name)/"session"),host="unused")
        self.app=App(args)
        self.sent=[]
        self.app.send_control=self.sent.append
        self.app.mapper.prepare_motion=lambda:None
        self.app.mapper.release_motion=lambda:None
        self.app.last_received=time.monotonic()
        self.app.mapper.state["tracking"]="ok"
        self.app.control.update(armed=True,phase="idle")
        self.app.front_heartbeat("operator",True)

    def tearDown(self):self.temp.cleanup();self.gate_patch.stop()

    def step(self,command="1"):
        return self.app.action("step","operator",command,"w")

    def test_same_command_is_idempotent(self):
        first=self.step()
        self.assertEqual(self.step(),first)
        self.assertEqual(len(self.sent),1)

    def test_busy_command_is_not_queued(self):
        self.step()
        with self.assertRaisesRegex(ValueError,"不排队"):self.step("2")
        self.assertEqual(len(self.sent),1)

    def test_concurrent_clicks_admit_only_one(self):
        def attempt(i):
            try:self.step(str(i));return True
            except ValueError:return False
        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            accepted=list(pool.map(attempt,range(20)))
        self.assertEqual(sum(accepted),1)
        self.assertEqual(len(self.sent),1)

    def test_stale_camera_rejects(self):
        self.app.last_received-=2
        with self.assertRaisesRegex(ValueError,"相机数据已过期"):self.step()
        self.assertEqual(self.sent,[])

    def test_focus_loss_rejects(self):
        self.app.front_heartbeat("operator",False)
        with self.assertRaisesRegex(ValueError,"控制焦点"):self.step()

    def test_tracking_loss_allows_manual_return_with_fresh_camera(self):
        self.app.mapper.state["tracking"]="lost"
        self.assertTrue(self.step()['accepted'])
        self.assertEqual(len(self.sent),1)

    def test_stop_remains_available_during_fault(self):
        self.app.uncertain=True
        self.app.last_received=None
        self.app.front_heartbeat("operator",False)
        self.assertTrue(self.app.action("stop","operator","stop1")["accepted"])
        self.assertEqual(self.sent,[{"op":"stop"}])

    def test_prepare_timeout_cancels_once_and_keeps_fault_until_verified(self):
        self.app.arm_pending=True
        self.app.pending_at=time.monotonic()-7
        def send(message):
            self.sent.append(message)
            if message['op']=='heartbeat':self.app.stop.set()
        self.app.send_control=send
        self.app.heartbeat_loop()
        self.assertEqual([m['op'] for m in self.sent],['stop','heartbeat'])
        self.assertFalse(self.app.arm_pending)
        self.assertTrue(self.app.uncertain)

    def test_replay_never_sends_motion(self):
        self.app.mode="replay"
        for op in ("arm","step","stop"):
            with self.assertRaisesRegex(ValueError,"回放模式"):
                self.app.action(op,"operator",op,"w")
        self.assertEqual(self.sent,[])

    def test_transport_failure_cannot_be_retried_as_accepted(self):
        def fail(_):raise OSError("disconnected")
        self.app.send_control=fail
        with self.assertRaises(OSError):self.step()
        self.assertFalse(self.step()["accepted"])
        self.assertTrue(self.app.uncertain)

    def test_focus_loss_during_mapper_preparation_never_sends_or_claims_motor_fault(self):
        self.app.mapper.prepare_motion=lambda:setattr(self.app,'ui_focused',False)
        with self.assertRaisesRegex(RuntimeError,'本次未下发'):self.step()
        self.assertEqual(self.sent,[])
        self.assertFalse(self.app.uncertain)
        self.assertFalse(self.step()['accepted'])

    def test_new_tab_can_claim_expired_unarmed_lease(self):
        self.app.control.update(armed=False,phase="disarmed")
        self.app.ui_seen-=2
        self.app.front_heartbeat("new",True)
        self.assertEqual(self.app.ui_owner,"new")

    def test_fall_interlock_rejects_prepare_and_step(self):
        self.app.motion_blocked=True
        for op in ('arm','step'):
            with self.assertRaisesRegex(ValueError,'步态已停用'):
                self.app.action(op,'operator',op,'w')
        self.assertEqual(self.sent,[])


if __name__=="__main__":unittest.main()
