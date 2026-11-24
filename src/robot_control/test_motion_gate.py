import json,tempfile,unittest
from pathlib import Path
from motion_gate import read_motion_interlock

class MotionGateTests(unittest.TestCase):
    def test_missing_interlock_blocks(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertTrue(read_motion_interlock(str(Path(tmp)/'missing.json'))['blocked'])
    def test_malformed_value_blocks(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'gate.json';p.write_text(json.dumps({'blocked':'false'}),encoding='utf-8')
            self.assertTrue(read_motion_interlock(str(p))['blocked'])
    def test_persisted_fall_blocks(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'gate.json';p.write_text(json.dumps({'blocked':True,'reason':'reported fall'}),encoding='utf-8')
            self.assertTrue(read_motion_interlock(str(p))['blocked'])
if __name__=='__main__':unittest.main()
