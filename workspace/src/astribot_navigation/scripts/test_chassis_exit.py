#!/usr/bin/env python3
"""Check fake chassis signal teardown on private topics; never enable a backend."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='125' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise RuntimeError('Requires localhost Domain 125')
    import rclpy
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_prefix
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    root=Path(args.output);root.mkdir(parents=True,exist_ok=False)
    executable=Path(get_package_prefix('towergo_real'))/'lib/towergo_real/chassis'
    rclpy.init();node=rclpy.create_node('chassis_exit_test');report={'hardware_used':False,'enabled':False,'signals':[]}
    try:
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            topic='/towergo_exit_test_'+str(os.getpid())+'_'+str(int(sig))
            messages=[];sub=node.create_subscription(String,topic+'/status',lambda m:messages.append(m.data),10)
            logfile=root/(sig.name+'.log')
            with logfile.open('w') as log:
                process=subprocess.Popen([str(executable),'--ros-args','-p','backend:=fake','-r','__node:=exit_test_chassis',
                    '-r','/towergo/control_status:='+topic+'/status','-r','/towergo/cmd_vel_teleop:='+topic+'/cmd',
                    '-r','/towergo/chassis/enable:='+topic+'/enable','-r','/towergo/chassis/disable:='+topic+'/disable'],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    end=time.monotonic()+10
                    while len(messages)<3 and time.monotonic()<end and process.poll() is None:
                        rclpy.spin_once(node,timeout_sec=.1)
                    assert len(messages)>=3,'No ready status'
                    assert json.loads(messages[-1])['state']=='disabled'
                    os.killpg(process.pid,sig);process.wait(timeout=5)
                    assert process.returncode==0,(sig.name,process.returncode)
                finally:
                    if process.poll() is None:os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
                    node.destroy_subscription(sub)
            text=logfile.read_text()
            assert not any(word in text for word in ('Destroyable','Traceback','exception was never retrieved')),text
            report['signals'].append({'signal':sig.name,'exit_code':process.returncode,'clean':True})
        report['passed']=True
        (root/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
