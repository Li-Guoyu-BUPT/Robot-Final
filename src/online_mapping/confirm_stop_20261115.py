import json,shlex
from pathlib import Path
from capture import connect
c=connect('10.29.201.125')
try:
    remote="import json,os,rospy,rosgraph,xmlrpclib\nfrom bodyhub.srv import SrvString,SrvTLSstring,SrvState\nrospy.init_node('single_step_stop_confirm',anonymous=True)\ns=rospy.ServiceProxy('/MediumSize/BodyHub/GetStatus',SrvString)\nm=rospy.ServiceProxy('/MediumSize/BodyHub/GetMasterID',SrvTLSstring)\nout={'state_before':s('read_only').data,'owner':m('read_only').data}\nif out['owner']==88 and out['state_before']=='walking':\n    out['stop_response']=rospy.ServiceProxy('/MediumSize/BodyHub/StateJump',SrvState)(88,'stop').stateRes\nmaster=rosgraph.Master(rospy.get_name())\npubs,subs,services=master.getSystemState()\napi=xmlrpclib.ServerProxy(os.environ.get('ROS_MASTER_URI','http://localhost:11311'))\nremoved=[]\nfor node in set(n for t,ns in pubs for n in ns if n.startswith('/roban_manual_single_step_10836_')):\n    uri=master.lookupNode(node)\n    for topic,nodes in pubs:\n        if node in nodes: api.unregisterPublisher(node,topic,uri)\n    for topic,nodes in subs:\n        if node in nodes: api.unregisterSubscriber(node,topic,uri)\n    removed.append(node)\nout.update(state_after=s('read_only').data,removed=removed,gait_publishers=[n for t,ns in master.getSystemState()[0] if t=='/gaitCommand' for n in ns])\nprint(json.dumps(out))\n"
    cmd='source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && timeout 12s python -c '+shlex.quote(remote)
    _,o,e=c.exec_command('bash -lc '+shlex.quote(cmd),timeout=18)
    result={'stdout':o.read().decode(),'stderr':e.read().decode(),'exit_code':o.channel.recv_exit_status()}
    Path(r'C:\Users\ASUS\Desktop\机器人大作业\功能实现\01_场景重建\验证记录\2026-11-15\stop_confirmed.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    s=c.open_sftp()
    data=s.open('/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/impl/tcpros_service.py','rb').read()
    Path(r'C:\Users\ASUS\Desktop\机器人大作业\功能实现\01_场景重建\验证记录\2026-11-15\控制源核对\tcpros_service.py').write_bytes(data)
    print(json.dumps(result,indent=2))
finally:c.close()

