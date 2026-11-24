"""Metric feature registration with depth on both views and rigid RANSAC."""
import numpy as np
import cv2
import open3d as o3d

def rigid(source,target):
    a,b=source.mean(0),target.mean(0)
    u,_,vt=np.linalg.svd((source-a).T@(target-b))
    r=vt.T@u.T
    if np.linalg.det(r)<0:vt[-1]*=-1;r=vt.T@u.T
    t=np.eye(4);t[:3,:3]=r;t[:3,3]=b-r@a
    return t

def feature_transform(source,target,min_inliers=30):
    if source is None or target is None or min(len(source[2]),len(target[2]))<min_inliers:return None
    a,auv,ad=source;b,buv,bd=target
    matches=cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(ad,bd,k=2)
    good=[x for pair in matches if len(pair)==2 for x,y in [pair] if x.distance<.72*y.distance]
    good=sorted(good,key=lambda m:m.distance)
    used=set();unique=[]
    for match in good:
        if match.trainIdx not in used:unique.append(match);used.add(match.trainIdx)
    if len(unique)<min_inliers:return None
    ai=np.array([m.queryIdx for m in unique]);bi=np.array([m.trainIdx for m in unique])
    x,y=a[ai].astype(float),b[bi].astype(float)
    rng=np.random.default_rng(917);best=None;best_count=0
    for _ in range(200):
        sample=rng.choice(len(x),3,replace=False)
        if np.linalg.norm(np.cross(x[sample[1]]-x[sample[0]],x[sample[2]]-x[sample[0]]))<.0001:continue
        t=rigid(x[sample],y[sample])
        error=np.linalg.norm(x@t[:3,:3].T+t[:3,3]-y,axis=1)
        inliers=error<.035;count=int(inliers.sum())
        if count>best_count:best,best_count=inliers,count
        if best_count>.8*len(x) and _>=20:break
    if best is None or best_count<min_inliers or best_count/len(x)<.4:return None
    for _ in range(3):
        t=rigid(x[best],y[best]);error=np.linalg.norm(x@t[:3,:3].T+t[:3,3]-y,axis=1)
        best=error<.03
        if best.sum()<min_inliers:return None
    rmse=float(np.sqrt(np.mean(error[best]**2)))
    # Repeated features on a small object must not determine the whole map pose.
    singular=np.linalg.svd(np.cov(x[best].T),compute_uv=False)
    cells=np.floor(auv[ai[best]]/[160.,160.]).astype(int)
    if singular[1]<.001 or len(np.unique(cells,axis=0))<4 or rmse>.022:return None
    return dict(transform=t,inliers=int(best.sum()),matches=len(x),rmse=rmse,source=x[best],target=y[best])

def make_cloud(full,intrinsic,voxel=.025):
    cloud=o3d.geometry.PointCloud.create_from_rgbd_image(full,intrinsic).voxel_down_sample(voxel)
    cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*3,max_nn=30))
    return cloud

def refine_pair(source,target,initial,max_distance=.04):
    try:
        result=o3d.pipelines.registration.registration_colored_icp(source,target,max_distance,initial,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(.97),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=25))
    except RuntimeError:return None
    if result.fitness<.45 or result.inlier_rmse>.024:return None
    return result
