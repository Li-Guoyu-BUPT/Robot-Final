import unittest
from teleop_policy import Policy,KeyEdges

class SingleStepTests(unittest.TestCase):
    def ready(self):
        p=Policy()
        p.heartbeat(1,10.,True)
        self.assertTrue(p.arm(10.))
        return p
    def test_hold_is_one_edge(self):
        e=KeyEdges(); e.poll([],True)
        self.assertEqual(e.poll(['w'],True),'w')
        self.assertTrue(all(e.poll(['w'],True) is None for _ in range(100)))
        e.poll([],True)
        self.assertEqual(e.poll(['w'],True),'w')
    def test_focus_reentry_requires_release(self):
        e=KeyEdges(); e.poll([],True); e.poll(['w'],False)
        self.assertIsNone(e.poll(['w'],True))
        e.poll([],True); self.assertEqual(e.poll(['w'],True),'w')
    def test_multikey_does_not_become_a_step_on_partial_release(self):
        e=KeyEdges(); e.poll([],True); e.poll(['w','a'],True)
        self.assertIsNone(e.poll(['w'],True))
    def test_repeated_gait_requests_consume_exactly_once(self):
        p=self.ready()
        self.assertEqual(p.submit('w',1,10.1),'accepted')
        self.assertEqual(p.take(10.2),(.05,0.,0.))
        self.assertTrue(all(p.take(10.3) is None for _ in range(100)))
    def test_busy_events_are_not_queued_or_replayed(self):
        p=self.ready(); p.submit('w',1,10.1); p.take(10.2)
        self.assertEqual(p.submit('a',2,10.3),'busy_not_queued')
        p.observe(1,10.4); self.assertTrue(p.observe(0,10.5))
        self.assertIsNone(p.take(10.6))
        self.assertEqual(p.submit('a',2,10.6),'duplicate')
    def test_initial_zero_does_not_complete(self):
        p=self.ready(); p.submit('w',1,10.1); p.take(10.2)
        self.assertFalse(p.observe(0,10.3)); self.assertEqual(p.phase,'executing')
        self.assertFalse(p.observe(1,10.4)); self.assertTrue(p.observe(0,10.5))
    def test_no_feedback_times_out_not_completes(self):
        p=self.ready(); p.submit('w',1,10.1); p.take(10.2)
        p.heartbeat(2,21.,True)
        self.assertEqual(p.fault(21.),'completion_feedback_timeout')
        self.assertEqual(p.phase,'executing')
    def test_stale_heartbeat_prevents_dispatch(self):
        p=self.ready(); p.submit('w',1,10.1)
        self.assertIsNone(p.take(11.))
        self.assertEqual(p.fault(11.),'heartbeat_or_camera_expired')
    def test_duplicate_heartbeat_does_not_extend_lease(self):
        p=self.ready(); self.assertFalse(p.heartbeat(1,10.7,True))
        self.assertFalse(p.fresh(11.))
    def test_stale_pending_step_does_not_dispatch(self):
        p=self.ready(); p.submit('w',1,10.1); p.heartbeat(2,14.,True)
        self.assertIsNone(p.take(14.)); self.assertEqual(p.fault(14.),'no_gait_request')
    def test_invalid_key_and_disarmed_never_dispatch(self):
        p=self.ready(); self.assertEqual(p.submit('run',1,10.1),'unknown_key')
        p.cancel(); self.assertEqual(p.submit('w',2,10.2),'not_armed')
        self.assertIsNone(p.take(10.2))
    def test_clock_rollback_never_dispatches(self):
        p=self.ready(); p.submit('w',1,10.1); self.assertIsNone(p.take(9.))
    def test_next_step_only_after_feedback(self):
        p=self.ready(); p.submit('w',1,10.1); p.take(10.2)
        p.observe(1,10.3); p.observe(0,10.4)
        self.assertEqual(p.submit('w',2,10.5),'accepted')
        self.assertEqual(p.take(10.6),(.05,0.,0.))

if __name__=='__main__': unittest.main()

