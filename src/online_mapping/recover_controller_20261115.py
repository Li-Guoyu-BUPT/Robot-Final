import json,shlex
from pathlib import Path
from capture import connect
remote="""import json,os,signal,rospy,rosgraph,xmlrpclib,time
from bodyhub.srv import SrvString,SrvTLSstring,SrvState
rospy.init_node('single_step_recovery',anonymous=True)
status=rospy.ServiceProxy('/MediumSize/BodyHub/GetStatus',SrvString)
owner=rospy.ServiceProxy('/MediumSize/BodyHub/GetMasterID',SrvTLSstring)
out={'owner_before':owner('read_only').data,'state_before':status('read_only').data}
pid=10836
expected='/tmp/roban_single_keys_07ed88f87a9c460ea0aa6cf210f626d3/robot_teleop.py'
cmd=open('/proc/%s/cmdline'%pid,'rb').read()
if expected not in cmd: raise RuntimeError('PID identity mismatch')
master=rosgraph.Master(rospy.get_name())
pubs,subs,services=master.getSystemState()
old=[]
for node in set(n for t,ns in pubs for n in ns if n.startswith('/roban_manual_single_step_')):
    uri=master.lookupNode(node)
    if xmlrpclib.ServerProxy(uri).getPid(rospy.get_name())[2]==pid: old.append((node,uri))
os.kill(pid,signal.SIGKILL)
api=xmlrpclib.ServerProxy(rospy.get_master_uri())
for node,uri in old:
    for topic,nodes in pubs:
        if node in nodes: api.unregisterPublisher(node,topic,uri)
    for topic,nodes in subs:
        if node in nodes: api.unregisterSubscriber(node,topic,uri)
out['killed_only_own_controller']=pid
out['removed_registrations']=[n for n,u in old]
if owner('read_only').data==88 and status('read_only').data=='walking':
    out['stop_response']=rospy.ServiceProxy('/MediumSize/BodyHub/StateJump',SrvState)(88,'stop').stateRes
out['state_after']=status('read_only').data
out['owner_after']=owner('read_only').data
out['gait_publishers']=[n for t,ns in master.getSystemState()[0] if t=='/gaitCommand' for n in ns]
print(json.dumps(out))
"""
c=connect('10.29.201.125')
try:
    cmd='source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && timeout 12s python -c '+shlex.quote(remote)
    _,o,e=c.exec_command('bash -lc '+shlex.quote(cmd),timeout=18)
    result={'stdout':o.read().decode(),'stderr':e.read().decode(),'exit_code':o.channel.recv_exit_status()}
    Path(r'C:\Users\ASUS\Desktop\机器人大作业\功能实现\01_场景重建\验证记录\2026-11-15\second_step_recovery.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
finally:c.close()

