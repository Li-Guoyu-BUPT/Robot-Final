import json,shlex
from pathlib import Path
from capture import connect
c=connect('10.29.201.125')
commands={
'status':"source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && timeout 3s rosservice call /MediumSize/BodyHub/GetStatus \"str: 'diagnose'\"",
'owner':"source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && timeout 3s rosservice call /MediumSize/BodyHub/GetMasterID \"str: 'diagnose'\"",
'logs':"find /home/lemon/.ros/log/latest -maxdepth 1 -name '*BodyHub*' -type f -exec tail -n 20 {} \\;",
'processes':"ps -eo pid,stat,pcpu,comm,args | grep -E '[B]odyHubNode|[r]obot_teleop.py'"
}
out={}
try:
    for name,cmd in commands.items():
        _,o,e=c.exec_command('bash -lc '+shlex.quote(cmd),timeout=8)
        text=o.read().decode(errors='replace')
        error=e.read().decode(errors='replace')
        out[name]={'stdout':text,'stderr':error,'exit':o.channel.recv_exit_status()}
    Path(r'C:\Users\ASUS\Desktop\机器人大作业\功能实现\01_场景重建\验证记录\2026-11-15\second_step_diagnostic.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    print(json.dumps(out,indent=2))
finally:c.close()

