"""Copy vendor reference source for review, never run it."""
import argparse,json
from pathlib import Path
from capture import connect
def main():
    p=argparse.ArgumentParser();p.add_argument('--out',required=True);a=p.parse_args();out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    relative=['bodyhub/CMakeLists.txt','leju_lib_pkg/src/motion/example/walking_example.py',
      'ros_actions_node/scripts/check_gait/check_walking.py','ros_actions_node/scripts/check_gait/gait.py',
      'leju_test_nodes/scripts/walking_path_test/walking_path_test.py']
    client=connect('10.29.201.125');manifest=[]
    try:
        sftp=client.open_sftp()
        for name in relative:
            target=out/'reference'/name;target.parent.mkdir(parents=True,exist_ok=True)
            try:sftp.get('/home/lemon/robot_ros_application/catkin_ws/src/'+name,str(target));manifest.append({'file':name,'copied':True})
            except OSError as exc:manifest.append({'file':name,'error':str(exc)})
        sftp.close()
        (out/'reference_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(manifest))
    finally:client.close()
if __name__=='__main__':main()
