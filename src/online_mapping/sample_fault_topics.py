"""Subscribe only; no motor service or publishers."""
import json,shlex,argparse
from pathlib import Path
from capture import connect
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);args=p.parse_args()
    code=r'''import rospy,json,math,time
from sensor_msgs.msg import BatteryState,Temperature,Imu
from std_msgs.msg import Float64
rospy.init_node('roban_fall_sensor_readonly',anonymous=True)
result={'robot_wall':time.time(),'motion_commands_sent':0}
for topic,kind in [('/MediumSize/SensorHub/BatteryState',BatteryState),('/MediumSize/SensorHub/Temperature',Temperature),('/MediumSize/SensorHub/Imu',Imu),('/contactState',Float64),('/stepPhase',Float64)]:
 try:
  m=rospy.wait_for_message(topic,kind,timeout=1.5)
  if kind==BatteryState:
   row={k:getattr(m,k) for k in ('voltage','current','charge','capacity','design_capacity','percentage','power_supply_status','power_supply_health','present')}
  elif kind==Temperature:row={'temperature':m.temperature,'variance':m.variance}
  elif kind==Imu:row={'orientation':[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w],'angular_velocity':[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],'linear_acceleration':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]}
  else:row={'data':m.data}
  if hasattr(m,'header'):row.update(stamp=m.header.stamp.to_sec(),frame_id=m.header.frame_id)
  result[topic]=row
 except Exception as e:result[topic]={'unavailable':str(e)}
print(json.dumps(result))
'''
    client=connect('10.29.201.125')
    try:
        env='source /opt/ros/kinetic/setup.bash && source ~/robot_ros_application/catkin_ws/devel/setup.bash && '
        _,stdout,stderr=client.exec_command('bash -lc '+shlex.quote(env+'timeout 12s python -u -c '+shlex.quote(code)),timeout=18)
        text=stdout.read().decode('utf-8',errors='replace');error=stderr.read().decode('utf-8',errors='replace');rc=stdout.channel.recv_exit_status()
        target=Path(args.out);target.mkdir(parents=True,exist_ok=True)
        (target/'sensor_samples.json').write_text(text,encoding='utf-8');(target/'sensor_samples_stderr.txt').write_text(error,encoding='utf-8')
        print('exit_code',rc);print(text)
    finally:client.close()
if __name__=='__main__':main()
