"""Use recorded latest frames to verify restored model alignment; no hardware."""
import json,sys,time
from pathlib import Path
import cv2,numpy as np
BASE=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(BASE/'重建'))
from live_mapper_v3 import LiveMapper
archive=BASE/'地图/质量优化/ghost_refined_validated_20261115'
frames=BASE/'数据/web_session_20261115_181713/frames'
m=LiveMapper();m.restore(archive,frames);m.set_paused(False)
paths=sorted(frames.glob('*.json'))[-30:]
results=[]
for p in paths:
    meta=json.loads(p.read_text(encoding='utf-8'));meta['motion_active']=False
    bgr=cv2.imdecode(np.frombuffer(p.with_suffix('.jpg').read_bytes(),np.uint8),1)
    dep=cv2.imdecode(np.frombuffer(p.with_suffix('.png').read_bytes(),np.uint8),-1)
    t=time.perf_counter();m.process(meta,bgr,dep);s=m.snapshot()
    results.append(dict(seq=meta['seq'],tracking=s['tracking'],model_fitness=s.get('model_fitness'),
        model_rmse_mm=s.get('model_rmse_mm'),fusion_blocked=s.get('fusion_blocked'),elapsed_ms=(time.perf_counter()-t)*1000))
report=dict(state=m.snapshot(),frames=results,source=str(frames),archive=str(archive))
(BASE/'验证记录/2026-11-15/model_guard_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
assert results[-1]['tracking']=='ok'
assert results[-1]['model_fitness']>.5
assert results[-1]['model_rmse_mm']<20
print(json.dumps(dict(last=results[-1],integrated=report['state']['integrated'],max_ms=max(x['elapsed_ms'] for x in results))))
