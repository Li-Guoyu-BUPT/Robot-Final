import unittest
import types
from unittest.mock import patch
import numpy as np
from multiview_registration import feature_transform
from live_mapper_v3 import LiveMapper

class MetricRegistrationTests(unittest.TestCase):
    def features(self):
        rng=np.random.default_rng(17)
        xyz=rng.uniform([-.7,-.4,.7],[.7,.4,2.],(150,3)).astype(np.float32)
        uv=rng.uniform([0,0],[639,479],(150,2)).astype(np.float32)
        des=rng.integers(0,256,(150,32),dtype=np.uint8)
        return rng,xyz,uv,des
    def test_metric_transform_survives_outliers(self):
        rng,x,uv,des=self.features();angle=np.radians(10)
        R=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
        y=x@R.T+[.04,-.01,.02]+rng.normal(0,.002,x.shape)
        y[:35]=rng.uniform(-1,1,(35,3))
        result=feature_transform((x,uv,des),(y,uv,des))
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result['transform'][:3,:3],R,atol=.002)
        np.testing.assert_allclose(result['transform'][:3,3],[.04,-.01,.02],atol=.002)
    def test_inconsistent_depth_is_rejected(self):
        rng,x,uv,des=self.features();y=rng.uniform(-1,1,x.shape)
        self.assertIsNone(feature_transform((x,uv,des),(y,uv,des)))
    def test_fusion_waits_for_stable_model_validation(self):
        mapper=LiveMapper();mapper.paused=False;mapper.settled=3;mapper.fusion_valid=True
        self.assertTrue(mapper.allow_fusion({}))
        mapper.fusion_valid=False;self.assertFalse(mapper.allow_fusion({}))
        mapper.fusion_valid=True;self.assertFalse(mapper.allow_fusion({'motion_active':True}))
        mapper.settled=2;self.assertFalse(mapper.allow_fusion({}))

    def test_stability_timer_restarts_after_motion(self):
        mapper=LiveMapper();pose=np.eye(4)
        for stamp in (0.,.5,1.,1.9):
            with patch('live_mapper_v3.time.monotonic',return_value=stamp):
                mapper.validate_pose({},None,None,None,pose)
            self.assertFalse(mapper.allow_fusion({}))
        with patch('live_mapper_v3.time.monotonic',return_value=2.1):
            mapper.validate_pose({},None,None,None,pose)
        self.assertTrue(mapper.allow_fusion({}))
        with patch('live_mapper_v3.time.monotonic',return_value=3.):
            mapper.validate_pose({'motion_active':True},None,None,None,pose)
        self.assertFalse(mapper.allow_fusion({}))
        with patch('live_mapper_v3.time.monotonic',return_value=3.1):
            mapper.validate_pose({},None,None,None,pose)
        self.assertFalse(mapper.allow_fusion({}))

    def test_metric_recovery_pose_direction(self):
        rng,x,uv,des=self.features();angle=np.radians(20)
        transform=np.eye(4)
        transform[:3,:3]=[[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]]
        transform[:3,3]=[.12,0,.03]
        y=x@transform[:3,:3].T+transform[:3,3]
        mapper=LiveMapper();reference=(x,uv,des)
        world=np.eye(4);world[:3,3]=[1,0,0]
        mapper.keyframes=[(None,world,reference)]
        mapper.features=lambda *args:(y,uv,des)
        mapper.alignment_quality=lambda *args:(.55,.012)
        result=mapper.recover(None,None,None,None)
        self.assertIsNotNone(result)
        np.testing.assert_allclose(result[0],world@np.linalg.inv(transform),atol=1e-5)
        self.assertEqual(mapper.state['relocalization'],'metric_history')

    def test_expansion_rejects_self_match(self):
        _,x,uv,des=self.features()
        mapper=LiveMapper();mapper.current_frame_id=('recording',7)
        mapper.keyframes=[(None,np.eye(4),(x,uv,des))]
        mapper.keyframe_ids=[('recording',7)]
        mapper.features=lambda *args:(x,uv,des)
        self.assertIsNone(mapper.local_expansion_pose(None,None,None,np.eye(4)))

    def test_expansion_requires_dense_depth_agreement(self):
        _,x,uv,des=self.features()
        mapper=LiveMapper();mapper.current_frame_id=('recording',8)
        mapper.keyframes=[(None,np.eye(4),(x,uv,des))]
        mapper.keyframe_ids=[('recording',7)];mapper.small_intrinsic=None
        mapper.features=lambda *args:(x,uv,des)
        mapper.alignment_quality=lambda *args:(.7,.012)
        self.assertIsNotNone(mapper.local_expansion_pose(None,None,None,np.eye(4)))
        mapper.alignment_quality=lambda *args:(.7,.04)
        self.assertIsNone(mapper.local_expansion_pose(None,None,None,np.eye(4)))

    def test_no_initial_map_overlap_needed_with_valid_adjacent_constraint(self):
        mapper=LiveMapper();mapper.model=object();pose=np.eye(4)
        registration=types.SimpleNamespace(fitness=0.,inlier_rmse=0.,transformation=pose)
        mapper.local_expansion_pose=lambda *args:pose.copy()
        with patch('live_mapper_v3.make_cloud',return_value=None),patch(
                'live_mapper_v3.o3d.pipelines.registration.registration_icp',return_value=registration):
            for stamp in (10.,10.5,12.1):
                with patch('live_mapper_v3.time.monotonic',return_value=stamp):
                    mapper.validate_pose({},None,None,None,pose)
            self.assertTrue(mapper.allow_fusion({}))
            self.assertEqual(mapper.state['fusion_validation'],'local_metric_expansion')
            mapper.local_expansion_pose=lambda *args:None
            with patch('live_mapper_v3.time.monotonic',return_value=14.):
                mapper.validate_pose({},None,None,None,pose)
            self.assertFalse(mapper.allow_fusion({}))

    def test_four_seconds_after_end_and_one_capture_per_motion(self):
        mapper=LiveMapper()
        with patch('live_mapper_v3.time.monotonic',return_value=10.):
            mapper.motion(True)
        with patch('live_mapper_v3.time.monotonic',return_value=20.):
            mapper.motion(False)
        mapper.settled=5;mapper.fusion_valid=True
        with patch('live_mapper_v3.time.monotonic',return_value=23.99):
            self.assertFalse(mapper.allow_fusion({}))
        with patch('live_mapper_v3.time.monotonic',return_value=24.):
            self.assertTrue(mapper.allow_fusion({}))
            mapper.capture_pending=False
            self.assertFalse(mapper.allow_fusion({}))
            mapper.motion(False) # Duplicate stop must not open another capture.
            self.assertFalse(mapper.allow_fusion({}))
            mapper.motion(True)
        self.assertTrue(mapper.capture_pending)
        self.assertFalse(mapper.allow_fusion({'motion_active':True}))
if __name__=='__main__':unittest.main()
