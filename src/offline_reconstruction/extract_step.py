"""Extract only a completed single-step interval; original recording remains immutable."""
import argparse,json,shutil
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('--session',required=True)
p.add_argument('--log',required=True)
p.add_argument('--event-id',type=int,required=True)
p.add_argument('--out',required=True)
a=p.parse_args()
session,out=Path(a.session),Path(a.out)
events=[]
for line in Path(a.log).read_text(encoding='utf-8').splitlines():
    try: r=json.loads(line)
    except ValueError: continue
    m=r.get('message',{})
    if r.get('role')=='robot' and m.get('event_id')==a.event_id: events.append(m)
sent=[e for e in events if e.get('event')=='step_published']
done=[e for e in events if e.get('event')=='step_completed']
accepted=[e for e in events if e.get('event')=='step_request' and e.get('result')=='accepted']
if len(sent)!=1 or len(done)!=1 or not accepted: raise ValueError('Need exactly one publication and one completion for this event')
start=accepted[0]['robot_wall']-3
end=done[0]['robot_wall']+3
rows=[]
for line in (session/'frame_manifest.jsonl').read_text().splitlines():
    try:m=json.loads(line)
    except ValueError:continue
    if start<=m['color_stamp']<=end: rows.append(m)
if not rows: raise ValueError('No camera frames match robot-clock interval')
if out.exists(): raise FileExistsError('Use a new derived dataset directory')
(out/'frames').mkdir(parents=True)
for m in rows:
    for ext in ('.jpg','.png','.json'):
        name=('%08d'%m['seq'])+ext
        shutil.copy2(session/'frames'/name,out/'frames'/name)
(out/'frame_manifest.jsonl').write_text(''.join(json.dumps(m)+'\n' for m in rows),encoding='utf-8')
provenance={'source_session':str(session),'source_log':a.log,'event_id':a.event_id,
            'robot_clock_start':start,'robot_clock_end':end,'frames':len(rows),
            'gait_commands':sent,'completion':done,'physical_displacement_measured':None,
            'note':'One vendor gait command, not a calibrated physical footfall count or measured 2cm displacement.'}
(out/'provenance.json').write_text(json.dumps(provenance,indent=2),encoding='utf-8')
print(json.dumps(provenance,indent=2))

