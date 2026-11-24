import json,shlex
from pathlib import Path
from capture import connect
root=Path(__file__).resolve().parent.parent
audit=json.loads((root/'验证记录'/'2026-11-15'/'控制源核对'/'audit.json').read_text(encoding='utf-8'))
expected={n:v['sha256'] for n,v in audit['sources'].items() if 'sha256' in v}
code="""import json,time,hashlib,rospy,rosnode,rosgraph
from bodyhub.srv import SrvString,SrvTLSstring
expected=EXPECTED
for name,sha in expected.items():
    data=open('/home/lemon/robot_ros_application/catkin_ws/src/'+name,'rb').read()
    if hashlib.sha256(data).hexdigest()!=sha: raise RuntimeError('Source changed: '+name)
rospy.init_node('manual_control_isolation',anonymous=True)
status=rospy.ServiceProxy('/MediumSize/BodyHub/GetStatus',SrvString)
owner=rospy.ServiceProxy('/MediumSize/BodyHub/GetMasterID',SrvTLSstring)
master=rosgraph.Master(rospy.get_name())
log=[]
def check():
    s=status('read_only')
    m=owner('read_only').data
    if s.data!='preReady' or m!=0 or s.poseQueueSize or s.jointQueueSize:
        raise RuntimeError('Abort isolation: state='+s.data+', owner='+str(m))
    return {'state':s.data,'owner':m,'pose_queue':s.poseQueueSize,'joint_queue':s.jointQueueSize}
def pubs():
    return [n for t,ns in master.getSystemState()[0] if t=='/gaitCommand' for n in ns]
try:
    for node in ['/head_toward_sound','/ros_aiui_node','/ros_socket_node','/joystick_handle_node','/slam_mapping_node']:
        before=check()
        ok,fail=rosnode.kill_nodes([node])
        if fail: raise RuntimeError('Cannot stop '+node)
        deadline=time.time()+4
        while node in pubs() and time.time()<deadline: time.sleep(.1)
        after=check()
        log.append({'node':node,'before':before,'after':after,'removed_from_gait_publishers':node not in pubs()})
        if node in pubs(): raise RuntimeError('Publisher remains: '+node)
    print(json.dumps({'steps':log,'final':check(),'remaining_publishers':pubs(),'motor_commands_sent':0}))
except Exception as e:
    print(json.dumps({'steps':log,'error':str(e),'motor_commands_sent':0}))
""".replace('EXPECTED',repr(expected))
c=connect('10.29.201.125')
try:
    cmd='source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && python -c '+shlex.quote(code)
    _,o,e=c.exec_command('bash -lc '+shlex.quote(cmd),timeout=30)
    text=o.read().decode()
    error=e.read().decode()
    lines=[line for line in text.splitlines() if line.startswith('{')]
    result=json.loads(lines[-1]) if lines else {'error':text+' '+error}
    result['stderr']=error
    (root/'验证记录'/'2026-11-19'/'control_isolation.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,indent=2,ensure_ascii=False))
finally:c.close()


