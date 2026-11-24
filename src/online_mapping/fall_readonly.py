"""Read-only post-fall audit: no initialization, torque changes or gait input."""
import argparse,json,shlex
from pathlib import Path
from capture import connect

def main():
    p=argparse.ArgumentParser();p.add_argument('--host',default='10.29.201.125');p.add_argument('--out',required=True);a=p.parse_args()
    out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    code=r'''import os,json,time,rospy,rosgraph,subprocess
from bodyhub.srv import SrvString,SrvTLSstring
def emit(stage,value):
 print(json.dumps({'stage':stage,'robot_wall':time.time(),'value':value}));import sys;sys.stdout.flush()
rospy.init_node('roban_fall_readonly_audit',anonymous=True)
s=rospy.ServiceProxy('/MediumSize/BodyHub/GetStatus',SrvString)('read_only')
m=rospy.ServiceProxy('/MediumSize/BodyHub/GetMasterID',SrvTLSstring)('read_only')
emit('bodyhub',dict((k,getattr(s,k)) for k in s.__slots__))
emit('master_id',m.data)
master=rosgraph.Master(rospy.get_name());system=master.getSystemState()
emit('gait_publishers',[n for t,nodes in system[0] if t=='/gaitCommand' for n in nodes])
emit('topics',master.getPublishedTopics('/'))
emit('services',[x[0] for x in system[2]])
root=os.path.expanduser('~/robot_ros_application/catkin_ws/src')
paths=[]
for folder,dirs,files in os.walk(root):
 dirs[:]=[d for d in dirs if d not in ('.git','build','devel')]
 for name in files:
  low=name.lower()
  if name.endswith(('.cpp','.h','.hpp','.yaml','.py')) and any(x in low for x in ('walking','gait','lipm','imu','balance')):
   paths.append(os.path.join(folder,name))
emit('source_candidates',paths[:160])
try:
 r=subprocess.check_output(['timeout','4','rosservice','call','/MediumSize/BodyHub/PresentTemperature'],stderr=subprocess.STDOUT)
 emit('servo_temperature',r)
except Exception as e:emit('servo_temperature_unavailable',str(e))
emit('motion_commands_sent',0)
'''
    client=connect(a.host)
    try:
        env='source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && '
        command=env+'timeout 18s python -u -c '+shlex.quote(code)
        _,stdout,stderr=client.exec_command('bash -lc '+shlex.quote(command),timeout=25)
        output=stdout.read().decode('utf-8',errors='replace');error=stderr.read().decode('utf-8',errors='replace')
        rc=stdout.channel.recv_exit_status()
        (out/'readonly_stdout.jsonl').write_text(output,encoding='utf-8');(out/'readonly_stderr.txt').write_text(error,encoding='utf-8')
        rows=[]
        for line in output.splitlines():
            try:rows.append(json.loads(line))
            except ValueError:pass
        (out/'readonly_audit.json').write_text(json.dumps({'exit_code':rc,'events':rows,'motion_commands_sent':0},ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({'exit_code':rc,'events':[r for r in rows if r['stage'] in ('bodyhub','master_id','gait_publishers','servo_temperature','servo_temperature_unavailable','motion_commands_sent')]},ensure_ascii=False,indent=2))
        candidates=next((r['value'] for r in rows if r['stage']=='source_candidates'),[])
        selected=[x for x in candidates if x.endswith(('.cpp','.h','.hpp')) and any(w in x.lower() for w in ('walking','gait','lipm'))]
        if selected:
            sftp=client.open_sftp()
            manifest=[]
            for remote in selected[:45]:
                relative=remote.split('/catkin_ws/src/',1)[-1];target=out/'source'/relative;target.parent.mkdir(parents=True,exist_ok=True)
                sftp.get(remote,str(target));manifest.append({'remote':remote,'local':str(target)})
            sftp.close();(out/'source_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
            print('Copied read-only source files:',len(manifest))
    finally:client.close()
if __name__=='__main__':main()
