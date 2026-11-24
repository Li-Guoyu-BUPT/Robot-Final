"""Offline pose graph refinement and re-fusion of an archived mapping run."""
import os
os.environ.setdefault('OMP_NUM_THREADS','8')
import argparse,json,sys,time
from pathlib import Path
import cv2,numpy as np,open3d as o3d
BASE=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(BASE/'重建'))
from reconstruct import export_cloud
from live_mapper_v2 import LiveMapper,rotation_degrees
from multiview_registration import feature_transform,make_cloud,refine_pair

def frame(row,mapper):
    stem=Path(row['meta']['_frames_source'])/('%08d'%row['seq'])
    bgr=cv2.imdecode(np.frombuffer(stem.with_suffix('.jpg').read_bytes(),np.uint8),1)
    depth=cv2.imdecode(np.frombuffer(stem.with_suffix('.png').read_bytes(),np.uint8),-1)
    return mapper.prepare(row['meta'],bgr,depth)

def select_records(records):
    groups=[];group=[];previous=None
    for r in records:
        split=False
        if previous is not None and not r['meta'].get('motion_active') and not previous['meta'].get('motion_active'):
            delta=np.linalg.inv(np.asarray(previous['T_world_camera']))@np.asarray(r['T_world_camera'])
            split=(r['stamp']-previous['stamp']>1. and (np.linalg.norm(delta[:3,3])>.025 or rotation_degrees(delta)>3.))
        if split and group:groups.append(group);group=[]
        if r['meta'].get('motion_active'):
            if group:groups.append(group);group=[]
        elif r['integrated']:group.append(r)
        previous=r
    if group:groups.append(group)
    selected=[]
    for group in groups:
        # Two settled observations per stationary run, not hundreds of noisy copies.
        for index in sorted(set([len(group)//2,len(group)-1])):selected.append(group[index])
    if len(selected)>70:selected=[selected[i] for i in np.linspace(0,len(selected)-1,70,dtype=int)]
    return selected

def refine(archive,output,progress=lambda **data:None):
    archive,output=Path(archive),Path(output);output.mkdir(parents=True,exist_ok=False)
    records=json.loads((archive/'trajectory.json').read_text(encoding='utf-8'))
    selected=select_records(records)
    if len(selected)<3:raise ValueError('可用稳定视角不足3个，先补采不同方向')
    mapper=LiveMapper();mapper.DEPTH_MAX=3.5
    views=[];begin=time.time()
    for i,row in enumerate(selected):
        small,k,full,intrinsic,valid=frame(row,mapper)
        views.append(dict(row=row,small=small,k=k,full=full,intrinsic=intrinsic,
            features=mapper.features(full,intrinsic),cloud=make_cloud(full,intrinsic),pose=np.asarray(row['T_world_camera'])))
        progress(stage='load',current=i+1,total=len(selected))
    graph=o3d.pipelines.registration.PoseGraph()
    for v in views:graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(v['pose']))
    edges=[];loop_count=0;correspondences=[]
    for i,a in enumerate(views):
        for j in range(i+1,len(views)):
            b=views[j];adjacent=j==i+1
            feature=feature_transform(a['features'],b['features'])
            initial=np.linalg.inv(b['pose'])@a['pose']
            result=refine_pair(a['cloud'],b['cloud'],feature['transform']) if feature else None
            if result is not None:
                T=result.transformation
                x,y=feature['source'],feature['target']
                feature_rmse=float(np.sqrt(np.mean(np.sum((x@T[:3,:3].T+T[:3,3]-y)**2,axis=1))))
                if feature_rmse>max(.026,feature['rmse']*1.6):result=None
            if result is not None:
                info=o3d.pipelines.registration.get_information_matrix_from_point_clouds(a['cloud'],b['cloud'],.03,T)
                graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(i,j,T,info,uncertain=not adjacent))
                old_error=float(np.sqrt(np.mean(np.sum((x@initial[:3,:3].T+initial[:3,3]-y)**2,axis=1))))
                edges.append(dict(source=i,target=j,loop=not adjacent,inliers=feature['inliers'],
                    old_feature_residual_m=old_error,registered_feature_residual_m=feature_rmse,
                    fitness=result.fitness,rmse=result.inlier_rmse))
                correspondences.append((x.copy(),y.copy()))
                if not adjacent:loop_count+=1
            elif adjacent:
                # Keep connectivity with a weak prior; do not call it a measured edge.
                graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(i,j,initial,np.eye(6)*10,uncertain=False))
        progress(stage='register',current=i+1,total=len(views),loops=loop_count)
    if loop_count<2:raise ValueError('不足2条可靠非相邻约束，保留原图，不输出伪优化结果')
    o3d.pipelines.registration.global_optimization(graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        o3d.pipelines.registration.GlobalOptimizationOption(max_correspondence_distance=.03,edge_prune_threshold=.25,reference_node=0))
    for edge,(x,y) in zip(edges,correspondences):
        relative=np.linalg.inv(graph.nodes[edge['target']].pose)@graph.nodes[edge['source']].pose
        edge['optimized_feature_residual_m']=float(np.sqrt(np.mean(np.sum((x@relative[:3,:3].T+relative[:3,3]-y)**2,axis=1))))
    old_median=float(np.median([e['old_feature_residual_m'] for e in edges]))
    optimized_median=float(np.median([e['optimized_feature_residual_m'] for e in edges]))
    if optimized_median>old_median or optimized_median>.035:
        raise ValueError('全局约束残差没有可靠改善，保留原图')
    refined=[];corrections=[]
    for i,v in enumerate(views):
        pose=np.asarray(graph.nodes[i].pose)
        mapper.volume.integrate(v['full'],v['intrinsic'],np.linalg.inv(pose))
        row=dict(v['row']);row['T_world_camera']=pose.tolist();row['integrated']=True;refined.append(row)
        change=np.linalg.inv(v['pose'])@pose
        corrections.append(dict(seq=row['seq'],translation_m=float(np.linalg.norm(change[:3,3])),rotation_deg=rotation_degrees(change)))
        progress(stage='fuse',current=i+1,total=len(views))
    cloud=mapper.volume.extract_point_cloud()
    cloud,_=cloud.remove_statistical_outlier(nb_neighbors=16,std_ratio=2.)
    points=export_cloud(cloud,output/'map.ply')
    mesh=mapper.volume.extract_triangle_mesh();mesh.compute_vertex_normals()
    import shutil,tempfile
    with tempfile.TemporaryDirectory(prefix='roban_mesh_') as tmp:
        path=Path(tmp)/'mesh.ply'
        if not o3d.io.write_triangle_mesh(str(path),mesh):raise IOError('Mesh export failed')
        shutil.copy2(path,output/'mesh.ply')
    (output/'trajectory.json').write_text(json.dumps(refined),encoding='utf-8')
    report=dict(mode='pose_graph_refined_v3',input_archive=str(archive),input_records=len(records),
        stable_keyframes=len(views),measured_edges=len(edges),loop_edges=loop_count,points=points,
        triangles=len(mesh.triangles),depth_max=3.5,elapsed_s=time.time()-begin,corrections=corrections,
        edge_metrics=edges,loop_closure=True,limitation='Internal consistency, no external pose ground truth. Unobserved geometry is not filled.')
    report.update(old_median_feature_residual_m=old_median,optimized_median_feature_residual_m=optimized_median)
    (output/'map_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    (output/'pose_graph.json').write_text(json.dumps(dict(nodes=[n.pose.tolist() for n in graph.nodes],
        edges=[dict(source=e.source_node_id,target=e.target_node_id,transform=e.transformation.tolist(),uncertain=e.uncertain,confidence=e.confidence) for e in graph.edges])),encoding='utf-8')
    import datetime
    (output/'checkpoint.json').write_text(json.dumps(dict(algorithm='model_guarded_v3',time=datetime.datetime.now().isoformat(timespec='seconds'),
        points=points,session='optimized_'+archive.parent.name,frames_source=selected[-1]['meta']['_frames_source']),ensure_ascii=False),encoding='utf-8')
    progress(stage='done',output=str(output),points=points)
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('archive');p.add_argument('--out',required=True);args=p.parse_args()
    try:refine(args.archive,args.out,lambda **r:print(json.dumps(r),flush=True))
    except Exception as exc:
        import traceback
        folder=Path(args.out);folder.mkdir(parents=True,exist_ok=True)
        (folder/'optimization_error.txt').write_text(traceback.format_exc(),encoding='utf-8')
        print(json.dumps(dict(stage='failed',error=str(exc))),flush=True)
        sys.exit(1)
