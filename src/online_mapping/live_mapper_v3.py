"""Frame-to-model guarded fusion, metric revisit constraints and surface output."""
import json,struct,time
from pathlib import Path
import numpy as np
import cv2
import open3d as o3d
from live_mapper_v2 import LiveMapper as BaseMapper,rotation_degrees
from multiview_registration import feature_transform,make_cloud

class LiveMapper(BaseMapper):
    VERSION="model_guarded_v3"
    COMPATIBLE=(VERSION,"keyframe_filtered_v2","pose_graph_refined_v3")
    DEPTH_MAX=3.5
    ANCHOR_TRANSLATION=.03
    ANCHOR_ROTATION=3.

    def reset(self):
        super().reset()
        self.model=None;self.mesh_packet=b"";self.last_model_check=0.;self.last_metric_check=0.
        self.previous_candidate=None;self.settled=0;self.fusion_valid=True;self.last_model_valid=True
        self.stable_since=None
        self.keyframe_ids=[];self.current_frame_id=None
        self.capture_pending=True;self.capture_moving=False;self.motion_ended_at=None
        self.frame_age_s=0.
        self.state.update(mesh_revision=self.state['revision'],triangles=0,
            fusion_blocked=None,model_fitness=None,model_rmse_mm=None,algorithm=self.VERSION)

    def prepare(self,meta,bgr,depth):
        result=super().prepare(meta,bgr,depth)
        self.small_intrinsic=result[1]
        scale=meta['depth_scale'];k=meta['K'];h,w=depth.shape
        with self.lock:
            self.state.update(raw_depth_ratio=float(np.mean(depth>0)),
                usable_depth_ratio=float(np.mean((depth>=.25*scale)&(depth<=self.DEPTH_MAX*scale))),
                filtered_depth_ratio=result[-1],depth_max=self.DEPTH_MAX,
                fov_h=float(np.degrees(np.arctan(k[2]/k[0])+np.arctan((w-k[2])/k[0]))),
                fov_v=float(np.degrees(np.arctan(k[5]/k[4])+np.arctan((h-k[5])/k[4]))))
        return result

    def promote_anchor(self,small,full,intrinsic,pose):
        self.anchor=small;self.anchor_pose=pose.copy();self.anchor_features=self.features(full,intrinsic)
        if self.anchor_features is not None:
            self.keyframes.append((small,pose.copy(),self.anchor_features))
            self.keyframe_ids.append(self.current_frame_id)
            if len(self.keyframes)>80:
                self.keyframes=self.keyframes[:30]+self.keyframes[-50:]
                self.keyframe_ids=self.keyframe_ids[:30]+self.keyframe_ids[-50:]
        with self.lock:
            self.state.update(keyframes=len(self.keyframes),anchor_seq=None if self.current_frame_id is None else self.current_frame_id[1])

    def local_expansion_pose(self,small,full,intrinsic,candidate):
        features=self.features(full,intrinsic)
        # Only historical frames qualify. Recovery can promote the current
        # image; matching it to itself is not evidence for map expansion.
        for index in range(len(self.keyframes)-1,max(-1,len(self.keyframes)-9),-1):
            frame_id=self.keyframe_ids[index] if index<len(self.keyframe_ids) else None
            if frame_id is not None and frame_id==self.current_frame_id:continue
            anchor,pose,reference=self.keyframes[index]
            match=feature_transform(reference,features,min_inliers=40)
            if match is None or match['rmse']>.02:continue
            transform=match['transform'];proposed=pose@np.linalg.inv(transform)
            difference=np.linalg.inv(candidate)@proposed
            if np.linalg.norm(difference[:3,3])>.04 or rotation_degrees(difference)>2.:continue
            forward,fr=self.alignment_quality(anchor,small,
                self.small_intrinsic,transform)
            reverse,rr=self.alignment_quality(small,anchor,
                self.small_intrinsic,np.linalg.inv(transform))
            if min(forward,reverse)<.5 or max(fr,rr)>.025:continue
            with self.lock:
                self.state.update(expansion_reference_seq=None if frame_id is None else frame_id[1],
                    expansion_inliers=match['inliers'],expansion_depth_rmse_mm=max(fr,rr)*1000)
            return proposed
        return None

    def validate_pose(self,meta,small,full,intrinsic,candidate):
        if self.previous_candidate is None:stable=True
        else:
            difference=np.linalg.inv(self.previous_candidate)@candidate
            stable=np.linalg.norm(difference[:3,3])<.012 and rotation_degrees(difference)<.8
        self.previous_candidate=candidate.copy()
        self.settled=self.settled+1 if stable and not meta.get('motion_active') else 0
        self.fusion_valid=False;reason='等待连续稳定画面'
        now=time.monotonic()
        if self.settled:
            if self.stable_since is None:self.stable_since=now
        else:self.stable_since=None
        stable_seconds=0. if self.stable_since is None else now-self.stable_since
        with self.lock:self.state['stable_seconds']=stable_seconds
        if self.settled>=3 and stable_seconds>=2.:
            self.fusion_valid=True;reason=None
            if self.model is not None and now-self.last_model_check>=.7:
                self.last_model_check=now
                # Metric revisits use depth in BOTH images, not only 2D reprojection.
                choices=[]
                if self.keyframes and now-self.last_metric_check>4.:
                    self.last_metric_check=now
                    features=self.features(full,intrinsic)
                    ids=np.unique(np.linspace(0,len(self.keyframes)-1,min(6,len(self.keyframes)),dtype=int))
                    for index in ids:
                        _,pose,reference=self.keyframes[index]
                        match=feature_transform(reference,features,min_inliers=35)
                        if match is not None:
                            proposed=pose@np.linalg.inv(match['transform'])
                            disagreement=np.linalg.inv(candidate)@proposed
                            if np.linalg.norm(disagreement[:3,3])<.06 and rotation_degrees(disagreement)<3.:
                                choices.append((match['inliers'],proposed))
                if choices:
                    candidate=max(choices,key=lambda item:item[0])[1]
                source=make_cloud(full,intrinsic,.02)
                result=o3d.pipelines.registration.registration_icp(source,self.model,.045,candidate,
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(
                        o3d.pipelines.registration.HuberLoss(.012)),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=12))
                self.last_model_check=time.monotonic()
                shift=np.linalg.inv(candidate)@result.transformation
                # New views can contain mostly unseen geometry. Permit partial
                # model overlap only with an independent depth-to-depth match.
                coverage_ok=result.fitness>.5 or (bool(choices) and result.fitness>.25)
                good=(coverage_ok and result.inlier_rmse<.02 and
                    np.linalg.norm(shift[:3,3])<.06 and rotation_degrees(shift)<3.)
                self.last_model_valid=bool(good)
                with self.lock:self.state.update(model_fitness=result.fitness,model_rmse_mm=result.inlier_rmse*1000)
                if good:
                    candidate=result.transformation
                    with self.lock:self.state['fusion_validation']='global_model'
                else:
                    # Global model overlap is optional. A moving RGB-D camera
                    # extends the map through validated adjacent observations.
                    proposed=self.local_expansion_pose(small,full,intrinsic,candidate)
                    self.last_model_valid=proposed is not None
                    if proposed is not None:
                        candidate=proposed
                        with self.lock:self.state['fusion_validation']='local_metric_expansion'
                    else:self.fusion_valid=False;reason='缺少可靠的相邻关键帧约束，暂停融合；无需与初始地图重合'
            elif self.model is not None:
                self.fusion_valid=self.last_model_valid
                if not self.fusion_valid:reason='等待模型配准'
            # Stationary jitter is never integrated as a stream of separate surfaces.
            if self.fusion_valid and self.last_integrated_pose is not None:
                change=np.linalg.inv(self.last_integrated_pose)@candidate
                if np.linalg.norm(change[:3,3])<.008 and rotation_degrees(change)<.6:
                    candidate=self.last_integrated_pose.copy()
        with self.lock:self.state['fusion_blocked']=reason
        return candidate

    def allow_fusion(self,meta):
        waiting=0. if self.motion_ended_at is None else max(0.,4.-(time.monotonic()-self.motion_ended_at-self.frame_age_s))
        with self.lock:
            self.state.update(post_motion_wait_s=waiting,capture_pending=self.capture_pending,
                              sampling_mode='one_keyframe_four_seconds_after_motion')
            if not self.capture_pending:self.state['fusion_blocked']='本次关键帧已采集，等待下一次动作'
            elif waiting>0:self.state['fusion_blocked']='动作后等待4秒：剩余 %.1f 秒'%waiting
        return (self.capture_pending and waiting<=0 and not self.capture_moving and
                super().allow_fusion(meta) and self.fusion_valid and self.settled>=3)

    def motion(self,active):
        active=bool(active)
        if active:
            self.capture_pending=True;self.stable_since=None;self.settled=0;self.fusion_valid=False
        elif self.capture_moving:self.motion_ended_at=time.monotonic()
        self.capture_moving=active

    def process(self,meta,bgr,depth):
        self.frame_age_s=max(0.,time.time()-meta.get('pc_received_wall',time.time()))
        self.motion(meta.get('motion_active',False))
        self.current_frame_id=(meta.get('_frames_source'),meta['seq'])
        integrated=self.state['integrated']
        super().process(meta,bgr,depth)
        if self.state['integrated']>integrated:
            self.capture_pending=False
            with self.lock:
                self.state.update(capture_pending=False,last_mapping_keyframe_seq=meta['seq'],
                                  fusion_blocked='本次关键帧已采集，等待下一次动作')
        if self.state.get('tracking')=='lost':
            self.stable_since=None;self.settled=0;self.previous_candidate=None
            with self.lock:
                self.state.update(stable_seconds=0.,fusion_blocked='定位丢失：正在匹配历史关键帧，暂停融合')

    def can_promote(self,meta):
        # Tracking references must advance during a turn even while TSDF fusion
        # waits for settling. Otherwise a frozen model also freezes odometry.
        with self.lock:
            overlap=self.state.get('overlap') or 0.
            rmse=self.state.get('depth_rmse_mm')
        return overlap>.5 and rmse is not None and rmse<35.

    def recover(self,small,full,intrinsic,small_k):
        # Search the visual history, then verify a rigid transform using depth
        # in BOTH views. Never initialize a new world pose from a gait command.
        features=self.features(full,intrinsic)
        if features is not None:
            matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
            ranked=[]
            for index,(anchor,pose,reference) in enumerate(self.keyframes):
                pairs=matcher.knnMatch(reference[2],features[2],k=2)
                score=sum(1 for p in pairs if len(p)==2 and p[0].distance<.72*p[1].distance)
                if score>=30:ranked.append((score,index))
            for _,index in sorted(ranked,reverse=True)[:6]:
                anchor,pose,reference=self.keyframes[index]
                match=feature_transform(reference,features,min_inliers=30)
                if match is None:continue
                transform=match['transform']
                overlap,rmse=self.alignment_quality(anchor,small,small_k,transform)
                reverse,reverse_rmse=self.alignment_quality(small,anchor,small_k,np.linalg.inv(transform))
                # Independent 3D RANSAC plus dense depth agreement in both views.
                if min(overlap,reverse)<.25 or max(rmse,reverse_rmse)>.03:continue
                candidate=pose@np.linalg.inv(transform)
                if not np.isfinite(candidate).all():continue
                with self.lock:
                    self.state.update(relocalization='metric_history',relocalization_anchor=index,
                                      metric_inliers=match['inliers'],metric_rmse_mm=match['rmse']*1000)
                # Advance the tracking anchor immediately after geometric recovery;
                # TSDF still waits for continuous visual settling and model checks.
                self.promote_anchor(small,full,intrinsic,candidate)
                return candidate,max(overlap,reverse),max(rmse,reverse_rmse),match['inliers']
        return super().recover(small,full,intrinsic,small_k)

    def refresh_cloud(self):
        super().refresh_cloud()
        cloud=self.volume.extract_point_cloud().voxel_down_sample(.015)
        if len(cloud.points):
            if not cloud.has_normals():cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.075,max_nn=30))
            self.model=cloud
        mesh=self.volume.extract_triangle_mesh()
        vertices=np.asarray(mesh.vertices);colors=np.asarray(mesh.vertex_colors);triangles=np.asarray(mesh.triangles)
        # Portable WebGL payload: header, interleaved vertex/color float32, uint32 indices.
        if len(vertices) and len(triangles):
            packet=struct.pack('<II',len(vertices),triangles.size)+np.column_stack((vertices,colors)).astype('<f4').tobytes()+triangles.astype('<u4').tobytes()
        else:packet=b""
        with self.lock:
            self.mesh_packet=packet;self.state.update(mesh_revision=self.state['revision'],triangles=len(triangles))

    def save(self,folder):
        path=super().save(folder)
        report_path=Path(folder)/'map_report.json'
        report=json.loads(report_path.read_text(encoding='utf-8'))
        report['loop_closure']=bool(self.state.get('loop_closure'))
        report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        import tempfile,shutil
        mesh=self.volume.extract_triangle_mesh();mesh.compute_vertex_normals()
        with tempfile.TemporaryDirectory(prefix='roban_mesh_') as tmp:
            safe=Path(tmp)/'mesh.ply'
            if not o3d.io.write_triangle_mesh(str(safe),mesh):raise IOError('Mesh export failed')
            shutil.copy2(safe,Path(folder)/'mesh.ply')
        return path

    def restore(self,folder,frames_source):
        folder,frames_source=Path(folder),Path(frames_source)
        report=json.loads((folder/'map_report.json').read_text(encoding='utf-8'))
        if report.get('mode') not in self.COMPATIBLE:raise ValueError('存档格式不兼容')
        records=json.loads((folder/'trajectory.json').read_text(encoding='utf-8'))
        if not records:raise ValueError('存档没有轨迹')
        restored=LiveMapper()
        stable=[i for i,r in enumerate(records) if not r['meta'].get('motion_active')]
        anchor_index=stable[-1] if stable else len(records)-1
        # Preserve several corrected historical anchors for future revisits.
        anchor_ids=set(np.linspace(0,anchor_index,min(20,anchor_index+1),dtype=int))&set(stable)
        anchor_ids.add(anchor_index)
        for i,row in enumerate(records):
            if not row['integrated'] and i not in anchor_ids and i!=len(records)-1:continue
            source=Path(row['meta'].get('_frames_source',str(frames_source)))
            stem=source/('%08d'%row['seq'])
            bgr=cv2.imdecode(np.frombuffer(stem.with_suffix('.jpg').read_bytes(),np.uint8),1)
            depth=cv2.imdecode(np.frombuffer(stem.with_suffix('.png').read_bytes(),np.uint8),-1)
            small,_,full,k,_=restored.prepare(row['meta'],bgr,depth);pose=np.asarray(row['T_world_camera'])
            if row['integrated']:restored.volume.integrate(full,k,np.linalg.inv(pose))
            if i in anchor_ids:restored.promote_anchor(small,full,k,pose)
            if i==len(records)-1:
                restored.pose=pose
                restored.signature=(depth.shape,tuple(row['meta']['K']),row['meta'].get('frame_id'),row['meta']['depth_scale'])
        restored.records=records;restored.trajectory=[np.asarray(r['T_world_camera'])[:3,3].tolist() for r in records]
        restored.state.update(processed=len(records),integrated=sum(r['integrated'] for r in records),tracking='ok',loop_closure=bool(report.get('loop_closure')))
        restored.last_integrated_pose=restored.pose.copy();restored.last_integrated_time=time.monotonic();restored.refresh_cloud()
        with self.compute_lock,self.lock:
            revision=self.state['revision']+1
            keys=('volume','anchor','anchor_pose','anchor_features','keyframes','pose','signature','records','trajectory',
                'last_integrated_pose','last_integrated_time','packet','state','model','mesh_packet','keyframe_ids')
            for key in keys:setattr(self,key,getattr(restored,key))
            self.state.update(revision=revision,mesh_revision=revision)
            self.last_cloud_time=0.;self.paused=True;self.previous_candidate=None;self.settled=0;self.stable_since=None
            self.capture_pending=True;self.capture_moving=False;self.motion_ended_at=None
