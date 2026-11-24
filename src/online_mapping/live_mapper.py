"""Continuous RGB-D tracking and TSDF fusion, independent from motion requests."""
import os
os.environ.setdefault("OMP_NUM_THREADS", "8")
import json
import threading
import time
from pathlib import Path
import cv2
import numpy as np
import open3d as o3d

class LiveMapper:
    def __init__(self):
        self.compute_lock=threading.RLock()
        self.lock=threading.RLock()
        self.volume=o3d.pipelines.integration.ScalableTSDFVolume(
            .01,.04,o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        self.previous=None
        self.pose=np.eye(4)
        self.last_integrated_pose=None
        self.last_integrated_time=0.
        self.signature=None
        self.paused=False
        self.packet=b""
        self.trajectory=[]
        self.records=[]
        self.state={"tracking":"waiting","processed":0,"integrated":0,"failed":0,
                    "revision":0,"points":0,"display_points":0,"processing_ms":0.,"error":None}
        self.last_cloud_time=0.
        self.option=o3d.pipelines.odometry.OdometryOption()
        self.option.iteration_number_per_pyramid_level=o3d.utility.IntVector([10,5,3])

    def snapshot(self):
        with self.lock:
            result=dict(self.state)
            result["paused"]=self.paused
            result["trajectory"]=list(self.trajectory[-400:])
            return result

    def set_paused(self,value):
        with self.lock:self.paused=bool(value)

    def process(self,meta,bgr,depth):
        begin=time.perf_counter()
        with self.compute_lock:
            h,w=depth.shape
            k=meta["K"]
            signature=(w,h,tuple(k),meta.get("frame_id"))
            if self.signature is not None and self.signature!=signature:
                raise ValueError("Camera calibration changed; start a new mapping session")
            if depth.dtype!=np.uint16 or bgr.shape[:2]!=depth.shape or k[0]<=0 or k[4]<=0:
                raise ValueError("Invalid RGB-D shape/type/calibration")
            self.signature=signature
            rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
            sw,sh=w//2,h//2
            odom_k=o3d.camera.PinholeCameraIntrinsic(sw,sh,k[0]/2,k[4]/2,(k[2]+.5)/2-.5,(k[5]+.5)/2-.5)
            full_k=o3d.camera.PinholeCameraIntrinsic(w,h,k[0],k[4],k[2],k[5])
            small=o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(np.ascontiguousarray(cv2.resize(rgb,(sw,sh)))),
                o3d.geometry.Image(np.ascontiguousarray(cv2.resize(depth,(sw,sh),interpolation=cv2.INTER_NEAREST))),
                float(meta["depth_scale"]),3.5,True)
            ok=True
            delta=np.eye(4)
            if self.previous is not None:
                ok,delta,_=o3d.pipelines.odometry.compute_rgbd_odometry(
                    self.previous,small,odom_k,np.eye(4),
                    o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),self.option)
                angle=np.degrees(np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1)))
                ok=bool(ok and np.isfinite(delta).all() and np.linalg.norm(delta[:3,3])<.18 and angle<14.)
            with self.lock:
                paused=self.paused
                self.state["processed"]+=1
            if not ok:
                with self.lock:
                    self.state["tracking"]="lost"
                    self.state["failed"]+=1
                    self.state["error"]="位姿估计失败，地图融合已冻结；相机和原始记录继续"
                return
            self.pose=self.pose@np.linalg.inv(delta)
            self.previous=small
            now=time.monotonic()
            integrate=not paused
            if integrate and self.last_integrated_pose is not None:
                change=np.linalg.inv(self.last_integrated_pose)@self.pose
                angle=np.degrees(np.arccos(np.clip((np.trace(change[:3,:3])-1)/2,-1,1)))
                integrate=(now-self.last_integrated_time>=.6 or np.linalg.norm(change[:3,3])>.015 or angle>1.)
            integrate=bool(integrate)
            if integrate:
                full=o3d.geometry.RGBDImage.create_from_color_and_depth(
                    o3d.geometry.Image(np.ascontiguousarray(rgb)),
                    o3d.geometry.Image(np.ascontiguousarray(depth)),float(meta["depth_scale"]),3.5,False)
                self.volume.integrate(full,full_k,np.linalg.inv(self.pose))
                self.last_integrated_pose=self.pose.copy()
                self.last_integrated_time=now
            record={"seq":meta["seq"],"stamp":meta["color_stamp"],"T_world_camera":self.pose.tolist(),
                    "integrated":integrate}
            with self.lock:
                self.records.append(record)
                self.trajectory.append(self.pose[:3,3].tolist())
                self.state.update(tracking="ok",error=None,processing_ms=(time.perf_counter()-begin)*1000)
                if integrate:self.state["integrated"]+=1
            if integrate and now-self.last_cloud_time>=.8:
                cloud=self.volume.extract_point_cloud()
                full_count=len(cloud.points)
                sampled=cloud.voxel_down_sample(.02)
                points=np.asarray(sampled.points)
                colors=np.asarray(sampled.colors)
                if len(points)>80000:
                    chosen=np.linspace(0,len(points)-1,80000,dtype=np.int64)
                    points,colors=points[chosen],colors[chosen]
                packet=np.column_stack((points,np.clip(colors,0,1))).astype("<f4").tobytes() if len(points) else b""
                with self.lock:
                    self.packet=packet
                    self.state["points"]=full_count
                    self.state["display_points"]=len(points)
                    self.state["revision"]+=1
                self.last_cloud_time=now

    def save(self,folder):
        from reconstruct import export_cloud
        folder=Path(folder)
        folder.mkdir(parents=True,exist_ok=False)
        with self.compute_lock:
            cloud=self.volume.extract_point_cloud()
            if not len(cloud.points):
                raise ValueError("地图还是空的，不能保存")
            points=export_cloud(cloud,folder/"map.ply")
            with self.lock:
                trajectory=list(self.records)
                state=self.snapshot()
            (folder/"trajectory.json").write_text(json.dumps(trajectory,indent=2),encoding="utf-8")
            state.update(points_saved=points,world_frame="initial_color_optical_frame",
                         loop_closure=False,mode="continuous_rgbd_odometry_tsdf")
            (folder/"map_report.json").write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding="utf-8")
            return str(folder/"map.ply")

