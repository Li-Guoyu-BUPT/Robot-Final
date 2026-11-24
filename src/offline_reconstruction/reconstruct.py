"""Offline RGB-D fusion. Input: frames/{id}.jpg, .png, .json; output: PLY + trajectory."""
import os
os.environ.setdefault('OMP_NUM_THREADS', '8')
import argparse
import json
from pathlib import Path
import shutil
import tempfile
import time
import cv2
import numpy as np
import open3d as o3d


def load_image(path, flags):
    image = cv2.imdecode(np.frombuffer(Path(path).read_bytes(), np.uint8), flags)
    if image is None:
        raise ValueError('Cannot decode '+str(path))
    return image


def export_cloud(cloud, target):
    if cloud.has_colors():
        cloud.colors=o3d.utility.Vector3dVector(np.clip(np.asarray(cloud.colors),0,1))
    # Open3D 0.18 Windows legacy I/O cannot reliably use Unicode filenames.
    with tempfile.TemporaryDirectory(prefix='roban_map_') as temp:
        safe = Path(temp)/'map.ply'
        if not o3d.io.write_point_cloud(str(safe), cloud) or not safe.is_file():
            raise IOError('Open3D failed to export PLY')
        restored = o3d.io.read_point_cloud(str(safe))
        if len(restored.points) != len(cloud.points) or not len(restored.points):
            raise IOError('PLY read-back verification failed')
        shutil.copy2(safe, target)
    return len(restored.points)


def reconstruct(session, output, mode='rgbd', limit=0, depth_max=3.5):
    session, output = Path(session), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    paths = sorted((session/'frames').glob('*.json'))
    if limit:
        paths = paths[:limit]
    if not paths:
        raise ValueError('No RGB-D metadata under '+str(session/'frames'))
    volume = o3d.pipelines.integration.ScalableTSDFVolume(0.01, 0.04, o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    option = o3d.pipelines.odometry.OdometryOption()
    option.iteration_number_per_pyramid_level = o3d.utility.IntVector([15, 10, 5])
    trajectory, previous, previous_stamp = [], None, None
    pose = np.eye(4)
    signature = None
    started = time.perf_counter()
    for path in paths:
        meta = json.loads(path.read_text(encoding='utf-8-sig'))
        color = load_image(path.with_suffix('.jpg'), cv2.IMREAD_COLOR)
        depth = load_image(path.with_suffix('.png'), cv2.IMREAD_UNCHANGED)
        if depth.dtype != np.uint16 or color.shape[:2] != depth.shape:
            raise ValueError('Depth type/size mismatch at '+path.name)
        height, width = depth.shape
        k = meta['K']
        if (width, height) != (meta['width'], meta['height']) or k[0]<=0 or k[4]<=0:
            raise ValueError('CameraInfo dimensions/intrinsics invalid')
        current_signature = (width, height, tuple(k), meta.get('frame_id'))
        if signature is not None and signature != current_signature:
            raise ValueError('Camera model changed mid-session; split the dataset')
        signature = current_signature
        scale = float(meta['depth_scale'])
        if scale <= 0:
            raise ValueError('Depth scale must be positive')
        rgb = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        ih = o3d.camera.PinholeCameraIntrinsic(width, height, k[0], k[4], k[2], k[5])
        small_w, small_h = width//2, height//2
        odom_k = o3d.camera.PinholeCameraIntrinsic(small_w, small_h, k[0]/2, k[4]/2, (k[2]+.5)/2-.5, (k[5]+.5)/2-.5)
        small = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.ascontiguousarray(cv2.resize(rgb,(small_w,small_h)))), o3d.geometry.Image(np.ascontiguousarray(cv2.resize(depth,(small_w,small_h),interpolation=cv2.INTER_NEAREST))), scale, depth_max, True)
        full = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.ascontiguousarray(rgb)), o3d.geometry.Image(np.ascontiguousarray(depth)), scale, depth_max, False)
        stamp = float(meta['color_stamp'])
        delta = np.eye(4)
        good, reason = True, 'initial'
        t0 = time.perf_counter()
        if previous is not None and mode == 'rgbd':
            if stamp<=previous_stamp or stamp-previous_stamp>2.0:
                good, reason = False, 'timestamp gap; recapture with more overlap'
            else:
                good, delta, information = o3d.pipelines.odometry.compute_rgbd_odometry(previous, small, odom_k, np.eye(4), o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option)
                angle = np.degrees(np.arccos(np.clip((np.trace(delta[:3,:3])-1)/2,-1,1)))
                good = bool(good and np.isfinite(delta).all() and np.linalg.norm(delta[:3,3])<0.15 and angle<12)
                reason = 'accepted_increment' if good else 'odometry_rejected'
        elif mode == 'stationary':
            reason = 'operator_declared_static_identity_pose'
        if good:
            pose = pose @ np.linalg.inv(delta)
            volume.integrate(full, ih, np.linalg.inv(pose))
            previous, previous_stamp = small, stamp
        trajectory.append({'frame':path.stem, 'stamp':stamp, 'integrated':good, 'reason':reason,
                           'processing_ms':(time.perf_counter()-t0)*1000, 'T_world_camera':pose.tolist() if good else None})
    cloud = volume.extract_point_cloud()
    count = export_cloud(cloud, output/'map.ply')
    summary = {'input_frames':len(paths), 'integrated_frames':sum(x['integrated'] for x in trajectory),
               'reloaded_points':count, 'mode':mode, 'world_frame':'initial_color_optical_frame',
               'elapsed_s':time.perf_counter()-started, 'loop_closure':False,
               'limitation':'Incremental RGB-D odometry and TSDF; no global relocalization, loop closure or base-frame calibration.'}
    (output/'trajectory.json').write_text(json.dumps(trajectory,indent=2),encoding='utf-8')
    (output/'report.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('session')
    parser.add_argument('--out',required=True)
    parser.add_argument('--mode',choices=['rgbd','stationary'],default='rgbd')
    parser.add_argument('--limit',type=int,default=0)
    parser.add_argument('--depth-max',type=float,default=3.5)
    args=parser.parse_args()
    print(json.dumps(reconstruct(args.session,args.out,args.mode,args.limit,args.depth_max),indent=2))
