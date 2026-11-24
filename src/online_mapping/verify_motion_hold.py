"""Black-box process handshake test with repeated saved RGB-D; no robot I/O."""
import json,time
from pathlib import Path
from map_worker import MapperProxy
BASE=Path(__file__).resolve().parent.parent

def main():
    report=json.loads((BASE/'验证记录/2026-11-15/checkpoint_roundtrip.json').read_text(encoding='utf-8'))
    archive=Path(report['saved']['path']).parent
    frames=BASE/'数据派生/20261115_step_001/frames'
    p=sorted(frames.glob('*.json'))[-1]
    meta=json.loads(p.read_text(encoding='utf-8-sig'));meta['_frames_source']=str(frames)
    m=MapperProxy()
    try:
        m.restore(archive,frames);m.set_paused(False);before=m.snapshot()['integrated']
        m.prepare_motion()
        for _ in range(8):m.process(meta,None,None);time.sleep(.12)
        time.sleep(.3);held=m.snapshot()['integrated']
        assert held==before,('Fusion occurred during motion hold',before,held)
        m.release_motion()
        for _ in range(16):m.process(meta,None,None);time.sleep(.15)
        deadline=time.monotonic()+5
        while m.snapshot()['integrated']<=held and time.monotonic()<deadline:time.sleep(.1)
        after=m.snapshot()['integrated'];assert after>held,('Fusion did not resume',m.snapshot())
        result=dict(test='repeated saved RGB-D simulation; no physical action',before=before,held=held,after=after)
        (BASE/'验证记录/2026-11-15/motion_hold_test.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps(result))
    finally:m.shutdown()
if __name__=='__main__':main()
