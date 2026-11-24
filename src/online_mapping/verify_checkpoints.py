"""HTTP checkpoint roundtrip on a replay server; no robot connection."""
import json,re,urllib.request
from pathlib import Path
URL='http://127.0.0.1:8767'
def get(route):return json.load(urllib.request.urlopen(URL+route))
html=urllib.request.urlopen(URL).read().decode()
token=re.search(r'"token": "([^"]+)"',html).group(1)
def post(route,data=None):
    req=urllib.request.Request(URL+route,json.dumps(data or {}).encode(),{'Content-Type':'application/json','X-Control-Token':token})
    return json.load(urllib.request.urlopen(req,timeout=90))
before=get('/api/status')
assert before['mode']=='replay' and before['map']['points']>0
saved=post('/api/save');arch=get('/api/status')['archives'][0]
cleared=post('/api/map/clear');empty=get('/api/status')
assert cleared['backup'] and Path(cleared['backup']).is_file()
assert empty['map']['points']==0 and empty['map']['integrated']==0
restored=post('/api/map/restore',{'archive_id':arch['id']});after=get('/api/status')
assert after['map']['paused'] and after['map']['points']==before['map']['points']
assert after['map']['integrated']==before['map']['integrated']
result={'saved':saved,'automatic_backup':cleared['backup'],'clear_points':0,'restored_points':after['map']['points'],'restored_paused':True,'raw_source_preserved':True}
target=Path(__file__).resolve().parent.parent/'验证记录/2026-11-15/checkpoint_roundtrip.json'
target.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
