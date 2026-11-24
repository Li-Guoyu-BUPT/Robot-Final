"""Python2/3-compatible persisted motion fault latch; fail closed."""
import json
import os

def read_motion_interlock(path=None):
    path=path or os.path.join(os.path.dirname(os.path.abspath(__file__)),'motion_interlock.json')
    try:
        with open(path,'rb') as f:data=json.loads(f.read().decode('utf-8-sig'))
        if type(data.get('blocked')) is not bool:raise ValueError('Invalid interlock value')
        return data
    except Exception:
        return {'blocked':True,'reason':'Motion interlock is missing or unreadable'}
