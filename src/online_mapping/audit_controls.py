import hashlib,json,sys,shlex
from pathlib import Path
from capture import connect
root=Path(__file__).resolve().parent.parent
dest=root/'验证记录'/'2026-11-15'/'控制源核对'
dest.mkdir(parents=True,exist_ok=True)
files=['SLAM/scripts/slam_mapping_action.py','leju_lib_pkg/src/motion/bodyhub_client.py','ros_AIUI_node/scripts/head_toward_sound.py','ros_AIUI_node/scripts/AIui_node.py','ros_actions_node/scripts/lejufunc/client_logger.py','ros_socket_node/scripts/transfer.py','ros_socket_node/launch/run.launch','joystickhandle/src/main.cpp','bodyhub/src/BodyHubNode.cpp']
c=connect('10.29.201.125')
try:
    s=c.open_sftp()
    report={}
    base='/home/lemon/robot_ros_application/catkin_ws/src/'
    reference=Path(r'C:\Users\ASUS\Desktop\机器人课程原始资料\机器人2\04_ROS源码\src')
    for name in files:
        try:
            data=s.open(base+name,'rb').read()
            out=dest/name
            out.parent.mkdir(parents=True,exist_ok=True)
            out.write_bytes(data)
            prior=reference/name
            report[name]={'sha256':hashlib.sha256(data).hexdigest(),'matches_reference':prior.is_file() and hashlib.sha256(prior.read_bytes()).digest()==hashlib.sha256(data).digest()}
        except Exception as e: report[name]={'error':str(e)}
    s.close()
    code="""import json,os,xmlrpclib,rosgraph
m=rosgraph.Master('/readonly_source_audit')
out={}
for n in ['/joystick_handle_node','/ros_socket_node','/slam_mapping_node','/ros_aiui_node','/head_toward_sound']:
    try:
        uri=m.lookupNode(n)
        pid=xmlrpclib.ServerProxy(uri).getPid('/readonly_source_audit')[2]
        out[n]={'pid':pid,'cmdline':open('/proc/%s/cmdline'%pid,'rb').read().replace(chr(0),' ')}
    except Exception as e: out[n]={'error':str(e)}
print(json.dumps(out))
"""
    cmd='source /opt/ros/kinetic/setup.bash && python -c '+shlex.quote(code)
    _,o,e=c.exec_command('bash -lc '+shlex.quote(cmd),timeout=15)
    processes=json.loads(o.read().decode())
    result={'sources':report,'processes':processes}
    (dest/'audit.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,indent=2,ensure_ascii=False))
finally:c.close()

