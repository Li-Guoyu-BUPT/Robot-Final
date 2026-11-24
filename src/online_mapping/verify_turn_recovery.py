"""Regression for the recorded 10-degree turn tracking failure; offline only."""
import json,sys,time
from pathlib import Path
import cv2,numpy as np
BASE=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(BASE/'重建'))
from live_mapper_v2 import LiveMapper

m=LiveMapper()
frames=BASE/'数据/web_session_20261115_180926/frames'
archive=BASE/'地图/web_session_20261115_180926/map_20261115_181258_088829'
m.restore(archive,frames)
results=[]
for i in list(range(344,381))+[400,500,700,1000]:
    p=frames/('%08d'%i);meta=json.loads(p.with_suffix('.json').read_text(encoding='utf-8'))
    color=cv2.imdecode(np.frombuffer(p.with_suffix('.jpg').read_bytes(),np.uint8),1)
    depth=cv2.imdecode(np.frombuffer(p.with_suffix('.png').read_bytes(),np.uint8),-1)
    meta['motion_active']=i<360
    m.process(meta,color,depth)
    s=m.snapshot();results.append(dict(seq=i,tracking=s['tracking'],overlap=s['overlap'],recoveries=s['recoveries']))
assert results[-1]['tracking']=='ok'
assert m.snapshot()['recoveries']>0
assert all(r['tracking']=='ok' for r in results if r['seq']>=365)
report={'test':'real recorded turn; no hardware command','archive':str(archive),'results':results,'state':m.snapshot()}
(BASE/'验证记录/2026-11-15/turn_recovery.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({k:m.snapshot().get(k) for k in ('tracking','processed','failed','recoveries','feature_inliers','overlap')}))
