import argparse
from pathlib import Path
import shutil
import tempfile
import numpy as np
import open3d as o3d
import cv2

parser=argparse.ArgumentParser()
parser.add_argument('map')
parser.add_argument('--preview',help='Save a preview PNG and exit without an interactive window')
args=parser.parse_args()
with tempfile.TemporaryDirectory(prefix='roban_view_') as folder:
    path=Path(folder)/'map.ply'
    shutil.copy2(args.map,path)
    cloud=o3d.io.read_point_cloud(str(path))
    if not len(cloud.points):
        raise ValueError('Map is empty')
cloud.transform(np.diag([1.,-1.,-1.,1.]))
if args.preview:
    vis=o3d.visualization.Visualizer()
    if not vis.create_window(width=1100,height=750,visible=False): raise RuntimeError('OpenGL preview unavailable')
    try:
        vis.add_geometry(cloud)
        vis.get_render_option().background_color=np.array([.07,.08,.10])
        vis.poll_events()
        vis.update_renderer()
        rgb=np.asarray(vis.capture_screen_float_buffer(do_render=True))
        ok,png=cv2.imencode('.png',cv2.cvtColor((np.clip(rgb,0,1)*255).astype(np.uint8),cv2.COLOR_RGB2BGR))
        if not ok: raise IOError('Preview encode failed')
        Path(args.preview).write_bytes(png.tobytes())
    finally:
        vis.destroy_window()
else:
    o3d.visualization.draw_geometries([cloud],window_name='ROBAN map - drag to rotate')
