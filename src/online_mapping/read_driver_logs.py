import json,shlex
from pathlib import Path
from capture import connect
c=connect('10.29.201.125')
try:
    cmd='python -c '+shlex.quote("import json,glob,os,collections\nout={'logs':{},'threads':[]}\nfor p in glob.glob('/home/lemon/.ros/log/latest/*BodyHub*'):\n    try:\n        f=open(p,'rb')\n        out['logs'][p]=''.join(collections.deque(f,maxlen=45))\n    except Exception as e: out['logs'][p]=str(e)\nfor t in glob.glob('/proc/3288/task/*'):\n    try: out['threads'].append({'tid':os.path.basename(t),'wait':open(t+'/wchan').read().strip(),'comm':open(t+'/comm').read().strip()})\n    except Exception:pass\nprint(json.dumps(out))\n")
    _,o,e=c.exec_command(cmd,timeout=10)
    text=o.read().decode()
    Path(r'C:\Users\ASUS\Desktop\机器人大作业\功能实现\01_场景重建\验证记录\2026-11-15\driver_log_diagnostic.json').write_text(text,encoding='utf-8')
    print(text)
finally:c.close()

