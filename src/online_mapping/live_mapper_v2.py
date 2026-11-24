"""Keyframe-anchored RGB-D tracking, filtered TSDF, restorable checkpoints."""
import os
os.environ.setdefault("OMP_NUM_THREADS", "8")
import json
import threading
import time
from pathlib import Path
import cv2
import numpy as np
import open3d as o3d

def rotation_degrees(matrix):
    return float(np.degrees(np.arccos(np.clip((np.trace(matrix[:3,:3])-1)/2,-1,1))))

class LiveMapper:
    VERSION="keyframe_filtered_v2"
    DEPTH_MAX=2.8
    ANCHOR_TRANSLATION=.08
    ANCHOR_ROTATION=8.
    def __init__(self):
        self.compute_lock=threading.RLock()
        self.lock=threading.RLock()
        self.state={"revision":0}
        self.option=o3d.pipelines.odometry.OdometryOption()
        self.option.iteration_number_per_pyramid_level=o3d.utility.IntVector([15,10,5])
        self.reset()

    def reset(self):
        with self.compute_lock,self.lock:
            revision=self.state["revision"]+1
            self.volume=o3d.pipelines.integration.ScalableTSDFVolume(.008,.032,o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
            self.anchor=None;self.anchor_pose=np.eye(4);self.pose=np.eye(4)
            self.anchor_features=None;self.keyframes=[]
            self.last_integrated_pose=None;self.last_integrated_time=0.
            self.signature=None;self.paused=False;self.packet=b""
            self.trajectory=[];self.records=[];self.last_cloud_time=0.
            self.state={"tracking":"waiting","processed":0,"integrated":0,"failed":0,
                "revision":revision,"points":0,"display_points":0,"processing_ms":0.,"error":None,
                "algorithm":self.VERSION,"overlap":None,"depth_rmse_mm":None,"motion_active":False}
            self.state["recoveries"]=0

    def snapshot(self):
        with self.lock:return dict(self.state,paused=self.paused,trajectory=list(self.trajectory[-400:]))

    def set_paused(self,value):
        with self.lock:self.paused=bool(value)

    def prepare(self,meta,bgr,depth):
        h,w=depth.shape;k=meta["K"]
        if depth.dtype!=np.uint16 or bgr.shape[:2]!=depth.shape or k[0]<=0 or k[4]<=0:
            raise ValueError("Invalid RGB-D shape/type/calibration")
        scale=float(meta["depth_scale"])
        if not 0<scale<100000:raise ValueError("Invalid depth scale")
        valid=(depth>=.25*scale)&(depth<=self.DEPTH_MAX*scale)
        filtered=cv2.medianBlur(depth,3)
        low=cv2.erode(np.where(valid,depth,65535).astype(np.uint16),np.ones((3,3),np.uint8))
        high=cv2.dilate(np.where(valid,depth,0).astype(np.uint16),np.ones((3,3),np.uint8))
        edge=(high.astype(np.int32)-low.astype(np.int32))>(.025+.006*(depth/scale)**2)*scale
        filtered[~valid|edge]=0
        rgb=np.ascontiguousarray(cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB))
        sw,sh=w//2,h//2
        intrinsic=o3d.camera.PinholeCameraIntrinsic(w,h,k[0],k[4],k[2],k[5])
        small_k=o3d.camera.PinholeCameraIntrinsic(sw,sh,k[0]/2,k[4]/2,(k[2]+.5)/2-.5,(k[5]+.5)/2-.5)
        full=o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(rgb),
            o3d.geometry.Image(np.ascontiguousarray(filtered)),scale,self.DEPTH_MAX,False)
        small=o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(np.ascontiguousarray(cv2.resize(rgb,(sw,sh),interpolation=cv2.INTER_AREA))),
            o3d.geometry.Image(np.ascontiguousarray(cv2.resize(filtered,(sw,sh),interpolation=cv2.INTER_NEAREST))),
            scale,self.DEPTH_MAX,True)
        return small,small_k,full,intrinsic,float(np.mean(filtered>0))

    @staticmethod
    def alignment_quality(source,target,intrinsic,transform):
        a=np.asarray(source.depth);b=np.asarray(target.depth)
        yy,xx=np.mgrid[0:a.shape[0]:3,0:a.shape[1]:3]
        z=a[yy,xx].ravel();u=xx.ravel();v=yy.ravel();valid=z>.25
        z,u,v=z[valid],u[valid],v[valid]
        if len(z)<300:return 0.,float("inf")
        k=intrinsic.intrinsic_matrix
        xyz=np.column_stack(((u-k[0,2])*z/k[0,0],(v-k[1,2])*z/k[1,1],z))
        xyz=xyz@transform[:3,:3].T+transform[:3,3]
        xyz=xyz[xyz[:,2]>.1]
        pu=np.rint(k[0,0]*xyz[:,0]/xyz[:,2]+k[0,2]).astype(int)
        pv=np.rint(k[1,1]*xyz[:,1]/xyz[:,2]+k[1,2]).astype(int)
        inside=(pu>=0)&(pu<b.shape[1])&(pv>=0)&(pv<b.shape[0])
        xyz,pu,pv=xyz[inside],pu[inside],pv[inside]
        target_z=b[pv,pu];error=np.abs(target_z-xyz[:,2])
        inlier=(target_z>.25)&(error<.035+.006*xyz[:,2]**2)
        overlap=float(np.sum(inlier))/len(z)
        rmse=float(np.sqrt(np.mean(error[inlier]**2))) if np.any(inlier) else float("inf")
        return overlap,rmse

    @staticmethod
    def features(full,intrinsic):
        gray=cv2.cvtColor(np.asarray(full.color),cv2.COLOR_RGB2GRAY)
        keys,des=cv2.ORB_create(nfeatures=2000,fastThreshold=10).detectAndCompute(gray,None)
        if des is None:return None
        uv=np.array([k.pt for k in keys],np.float32)
        pixels=np.rint(uv).astype(int);depth=np.asarray(full.depth)
        z=depth[pixels[:,1],pixels[:,0]]
        # The RGB-D image is already clipped to the active mapper's range.
        # A fixed base-class limit silently drops v3's 2.8--3.5 m features.
        good=(z>.25)&np.isfinite(z)
        uv,z,des=uv[good],z[good],des[good]
        k=intrinsic.intrinsic_matrix
        xyz=np.column_stack(((uv[:,0]-k[0,2])*z/k[0,0],(uv[:,1]-k[1,2])*z/k[1,1],z)).astype(np.float32)
        return xyz,uv,des

    def promote_anchor(self,small,full,intrinsic,pose):
        self.anchor=small;self.anchor_pose=pose.copy();self.anchor_features=self.features(full,intrinsic)
        if self.anchor_features is not None:
            self.keyframes.append((small,pose.copy(),self.anchor_features))
            self.keyframes=self.keyframes[-12:]

    def recover(self,small,full,intrinsic,small_k):
        gray=cv2.cvtColor(np.asarray(full.color),cv2.COLOR_RGB2GRAY)
        keys,des=cv2.ORB_create(nfeatures=2200,fastThreshold=10).detectAndCompute(gray,None)
        if des is None:return None
        uv=np.array([k.pt for k in keys],np.float32)
        matcher=cv2.BFMatcher(cv2.NORM_HAMMING)
        depth=np.asarray(full.depth);k=intrinsic.intrinsic_matrix
        for anchor,pose,features in reversed(self.keyframes):
            xyz,_,refdes=features
            matches=matcher.knnMatch(refdes,des,k=2)
            good=[a for pair in matches if len(pair)==2 for a,b in [pair] if a.distance<.72*b.distance]
            if len(good)<20:continue
            obj=xyz[[a.queryIdx for a in good]];pixels=uv[[a.trainIdx for a in good]]
            ok,rvec,tvec,inliers=cv2.solvePnPRansac(obj,pixels,k,None,iterationsCount=150,reprojectionError=3.,confidence=.999,flags=cv2.SOLVEPNP_EPNP)
            if not ok or inliers is None or len(inliers)<20 or len(inliers)/len(good)<.5:continue
            indices=inliers.ravel()
            rvec,tvec=cv2.solvePnPRefineLM(obj[indices],pixels[indices],k,None,rvec,tvec)
            R,_=cv2.Rodrigues(rvec);T=np.eye(4);T[:3,:3]=R;T[:3,3]=tvec.ravel()
            projected=obj[indices]@R.T+tvec.ravel()
            pix=np.rint(pixels[indices]).astype(int);measured=depth[pix[:,1],pix[:,0]]
            valid=measured>.25
            if np.sum(valid)<15 or np.median(np.abs(measured[valid]-projected[valid,2]))>.035:continue
            overlap,rmse=self.alignment_quality(anchor,small,small_k,T)
            candidate=pose@np.linalg.inv(T)
            if overlap>.4 and rmse<.03 and np.linalg.norm(candidate[:3,3]-self.pose[:3,3])<.6:
                return candidate,overlap,rmse,len(inliers)
        return None

    def validate_pose(self,meta,small,full,intrinsic,candidate):
        return candidate

    def allow_fusion(self,meta):
        return not self.paused and not meta.get("motion_active",False)

    def can_promote(self,meta):
        return not meta.get('motion_active')

    def process(self,meta,bgr,depth):
        begin=time.perf_counter()
        with self.compute_lock:
            signature=(depth.shape,tuple(meta["K"]),meta.get("frame_id"),meta["depth_scale"])
            if self.signature is not None and self.signature!=signature:
                raise ValueError("Camera calibration changed; start a new mapping segment")
            small,small_k,full,intrinsic,valid_ratio=self.prepare(meta,bgr,depth)
            ok=valid_ratio>.12;overlap,rmse=1.,0.;candidate=self.pose.copy()
            if self.anchor is not None and ok:
                initial=np.linalg.inv(self.pose)@self.anchor_pose
                ok,delta,_=o3d.pipelines.odometry.compute_rgbd_odometry(self.anchor,small,small_k,initial,
                    o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),self.option)
                if ok and np.isfinite(delta).all():
                    candidate=self.anchor_pose@np.linalg.inv(delta)
                    increment=np.linalg.inv(self.pose)@candidate
                    overlap,rmse=self.alignment_quality(self.anchor,small,small_k,delta)
                    ok=bool(np.linalg.norm(increment[:3,3])<.18 and rotation_degrees(increment)<16.
                            and overlap>.5 and rmse<.035)
                else:ok=False
            if not ok:
                recovered=self.recover(small,full,intrinsic,small_k)
                if recovered is not None:
                    candidate,overlap,rmse,inliers=recovered;ok=True
                    with self.lock:
                        self.state["recoveries"]+=1;self.state["feature_inliers"]=inliers
            with self.lock:
                self.state["processed"]+=1
                self.state.update(overlap=overlap,depth_rmse_mm=rmse*1000 if np.isfinite(rmse) else None,
                    motion_active=bool(meta.get("motion_active")),valid_depth_ratio=valid_ratio)
            if not ok:
                with self.lock:
                    self.state.update(tracking="lost",error="配准质量不足，已冻结融合；返回上一可靠视角或清除后重建")
                    self.state["failed"]+=1
                return
            candidate=self.validate_pose(meta,small,full,intrinsic,candidate)
            self.signature=signature;self.pose=candidate
            # A fixed anchor prevents pairwise stationary noise becoming a random walk.
            if self.anchor is None:
                self.promote_anchor(small,full,intrinsic,self.pose)
            else:
                shift=np.linalg.inv(self.anchor_pose)@self.pose
                if (np.linalg.norm(shift[:3,3])>self.ANCHOR_TRANSLATION or rotation_degrees(shift)>self.ANCHOR_ROTATION) and self.can_promote(meta):
                    self.promote_anchor(small,full,intrinsic,self.pose)
            now=time.monotonic()
            with self.lock:paused=self.paused
            integrate=self.allow_fusion(meta)
            if integrate and self.last_integrated_pose is not None:
                change=np.linalg.inv(self.last_integrated_pose)@self.pose
                integrate=bool(now-self.last_integrated_time>=1. or np.linalg.norm(change[:3,3])>.015 or rotation_degrees(change)>1.)
            if integrate:
                self.volume.integrate(full,intrinsic,np.linalg.inv(self.pose))
                self.last_integrated_pose=self.pose.copy();self.last_integrated_time=now
            record={"seq":meta["seq"],"stamp":meta["color_stamp"],"meta":dict(meta),
                    "T_world_camera":self.pose.tolist(),"integrated":bool(integrate)}
            with self.lock:
                self.records.append(record);self.trajectory.append(self.pose[:3,3].tolist())
                self.state.update(tracking="ok",error=None,processing_ms=(time.perf_counter()-begin)*1000)
                if integrate:self.state["integrated"]+=1
            if integrate and now-self.last_cloud_time>=.8:
                self.refresh_cloud();self.last_cloud_time=now

    def refresh_cloud(self):
        cloud=self.volume.extract_point_cloud();sampled=cloud.voxel_down_sample(.012)
        points=np.asarray(sampled.points);colors=np.asarray(sampled.colors)
        if len(points)>180000:
            chosen=np.linspace(0,len(points)-1,180000,dtype=np.int64);points,colors=points[chosen],colors[chosen]
        packet=np.column_stack((points,np.clip(colors,0,1))).astype("<f4").tobytes() if len(points) else b""
        with self.lock:
            self.packet=packet
            self.state.update(points=len(cloud.points),display_points=len(points),revision=self.state["revision"]+1)

    def save(self,folder):
        from reconstruct import export_cloud
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=False)
        with self.compute_lock:
            cloud=self.volume.extract_point_cloud()
            if not len(cloud.points):raise ValueError("地图还是空的，不能存档")
            points=export_cloud(cloud,folder/"map.ply")
            with self.lock:records=list(self.records);state=self.snapshot()
            (folder/"trajectory.json").write_text(json.dumps(records),encoding="utf-8")
            state.update(points_saved=points,world_frame="initial_color_optical_frame",loop_closure=False,mode=self.VERSION,depth_max=self.DEPTH_MAX)
            (folder/"map_report.json").write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding="utf-8")
            return str(folder/"map.ply")

    def restore(self,folder,frames_source):
        folder,frames_source=Path(folder),Path(frames_source)
        report=json.loads((folder/"map_report.json").read_text(encoding="utf-8"))
        if report.get("mode")!=self.VERSION:raise ValueError("该存档算法版本不同，只能查看PLY，不能继续融合")
        records=json.loads((folder/"trajectory.json").read_text(encoding="utf-8"))
        if not records:raise ValueError("存档没有轨迹")
        # Build separately: failed restore leaves current TSDF and tracker intact.
        restored=LiveMapper()
        anchor_index=max((i for i,r in enumerate(records) if not r["meta"].get("motion_active")),default=len(records)-1)
        for i,row in enumerate(records):
            if not row["integrated"] and i not in (len(records)-1,anchor_index):continue
            row_source=Path(row["meta"].get("_frames_source",str(frames_source)))
            stem=row_source/("%08d"%row["seq"])
            bgr=cv2.imdecode(np.frombuffer(stem.with_suffix(".jpg").read_bytes(),np.uint8),cv2.IMREAD_COLOR)
            depth=cv2.imdecode(np.frombuffer(stem.with_suffix(".png").read_bytes(),np.uint8),cv2.IMREAD_UNCHANGED)
            small,_,full,k,_=restored.prepare(row["meta"],bgr,depth)
            pose=np.asarray(row["T_world_camera"])
            if row["integrated"]:restored.volume.integrate(full,k,np.linalg.inv(pose))
            if i==anchor_index:restored.promote_anchor(small,full,k,pose)
            if i==len(records)-1:
                restored.pose=pose
                restored.signature=(depth.shape,tuple(row["meta"]["K"]),row["meta"].get("frame_id"),row["meta"]["depth_scale"])
        restored.records=records
        restored.trajectory=[np.asarray(r["T_world_camera"])[:3,3].tolist() for r in records]
        restored.state.update(processed=len(records),integrated=sum(r["integrated"] for r in records),tracking="ok")
        restored.last_integrated_pose=restored.pose.copy();restored.last_integrated_time=time.monotonic();restored.refresh_cloud()
        with self.compute_lock,self.lock:
            revision=self.state["revision"]+1
            for key in ("volume","anchor","anchor_pose","anchor_features","keyframes","pose","signature","records","trajectory","last_integrated_pose","last_integrated_time","packet","state"):
                setattr(self,key,getattr(restored,key))
            self.state["revision"]=revision;self.last_cloud_time=0.;self.paused=True
