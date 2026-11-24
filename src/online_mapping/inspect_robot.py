"""Read-only ROS audit and Python2 compilation of the proposed control adapter."""
import argparse
import json
from pathlib import Path
import shlex
import uuid
from capture import connect

parser=argparse.ArgumentParser()
parser.add_argument('--host',default='10.29.201.125')
parser.add_argument('--out',required=True)
args=parser.parse_args()
client=connect(args.host)
try:
    folder='/tmp/roban_stage1_audit_'+uuid.uuid4().hex
    sftp=client.open_sftp()
    sftp.mkdir(folder)
    for name in ('robot_teleop.py','teleop_policy.py'):
        sftp.put(str(Path(__file__).resolve().parent.parent/'遥控'/name),folder+'/'+name)
    sftp.close()
    code="""import json,rospy,rosgraph,subprocess
from bodyhub.srv import SrvString,SrvTLSstring
rospy.init_node('roban_stage1_readonly_audit',anonymous=True)
s=rospy.ServiceProxy('/MediumSize/BodyHub/GetStatus',SrvString)('read_only')
m=rospy.ServiceProxy('/MediumSize/BodyHub/GetMasterID',SrvTLSstring)('read_only')
publishers=rosgraph.Master(rospy.get_name()).getSystemState()[0]
p=[n for t,nodes in publishers if t=='/gaitCommand' for n in nodes]
rows=subprocess.check_output(['ps','-eo','pid=,args=']).splitlines()
agents=[r for r in rows if len(r.strip().split(None,1))==2 and r.strip().split(None,1)[1].startswith('python -u /tmp/roban_sensor_')]
print(json.dumps({'bodyhub_state':s.data,'master_id':m.data,'gait_publishers':p,'own_sensor_processes':agents,'motion_commands_sent':0,'controller_python2_syntax':'passed'}))
"""
    cmd='source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && python -m py_compile '+shlex.quote(folder+'/robot_teleop.py')+' '+shlex.quote(folder+'/teleop_policy.py')+' && python -c '+shlex.quote(code)
    _,stdout,stderr=client.exec_command('bash -lc '+shlex.quote(cmd),timeout=20)
    output=stdout.read().decode('utf-8')
    error=stderr.read().decode('utf-8')
    if stdout.channel.recv_exit_status(): raise RuntimeError(error)
    result=json.loads(output.strip().splitlines()[-1])
    Path(args.out).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
finally:
    client.close()
