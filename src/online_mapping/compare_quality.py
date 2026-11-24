"""Compare stationary drift on the same recorded frames, with no robot I/O."""
import argparse,json,time,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'重建'))
import cv2,numpy as np
from live_mapper import LiveMapper as Original
from live_mapper_v2 import LiveMapper as Improved

def main():
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('--start',type=int,default=2000);p.add_argument('--count',type=int,default=200);p.add_argument('--out',required=True);args=p.parse_args()
    paths=[p for p in sorted((Path(args.source)/'frames').glob('*.json')) if int(p.stem)>=args.start][:args.count]
    output=Path(args.out);output.mkdir(parents=True,exist_ok=True)
    result={"source":args.source,"first":paths[0].stem,"last":paths[-1].stem,"input_frames":len(paths),"test":"no-gait-command interval, not ground-truth accuracy"}
    for name,kind in [('original',Original),('improved',Improved)]:
        m=kind();positions=[];begin=time.perf_counter()
        for i,path in enumerate(paths):
            meta=json.loads(path.read_text(encoding='utf-8-sig'))
            bgr=cv2.imdecode(np.frombuffer(path.with_suffix('.jpg').read_bytes(),np.uint8),1)
            depth=cv2.imdecode(np.frombuffer(path.with_suffix('.png').read_bytes(),np.uint8),-1)
            # Same selected fusion frames for a fair fast offline comparison.
            m.set_paused(i%5!=0);m.last_integrated_time=0
            m.process(meta,bgr,depth)
            positions.append(m.pose[:3,3].copy())
        a=np.asarray(positions);state=m.snapshot();m.save(output/name)
        result[name]={"drift_m":float(np.linalg.norm(a[-1]-a[0])),"axis_span_m":np.ptp(a,axis=0).tolist(),
            "accepted":len(m.records),"failed":state['failed'],"integrated":state['integrated'],"elapsed_s":time.perf_counter()-begin}
        print(name,result[name],flush=True)
    (output/'comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
if __name__=='__main__':main()
