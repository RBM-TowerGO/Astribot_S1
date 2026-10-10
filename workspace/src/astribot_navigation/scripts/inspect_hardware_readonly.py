#!/usr/bin/env python3
"""Read ROS discovery, feedback, TF and selected parameters. No SDK or control RPCs."""
import argparse
import json
import math
import os
from pathlib import Path
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='25':raise RuntimeError('Read-only robot inspection requires Domain 25')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from rcl_interfaces.srv import ListParameters,GetParameters
    from tf2_msgs.msg import TFMessage
    from astribot_msgs.msg import RobotJointState
    from sensor_msgs.msg import Imu
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():raise RuntimeError('Refusing to overwrite prior inspection')
    rclpy.init();node=rclpy.create_node('towergo_hardware_readonly_inspection')
    report={'control_commands_sent':False,'sdk_created':False,'domain':25,'started_unix':time.time()}
    feedback=[];imu=[];transforms={};clients=[]
    def joint(m):
        feedback.append({'received_monotonic':time.monotonic(),'age_s':node.get_clock().now().nanoseconds*1e-9-(m.header.stamp.sec+m.header.stamp.nanosec*1e-9),
                         'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'position':list(m.position),'velocity':list(m.velocity)})
    def imu_cb(m):
        a=m.linear_acceleration;imu.append([a.x,a.y,a.z])
    def tf_cb(m):
        for t in m.transforms:
            p=t.transform.translation;q=t.transform.rotation
            transforms[t.header.frame_id+' -> '+t.child_frame_id]={'xyz':[p.x,p.y,p.z],'xyzw':[q.x,q.y,q.z,q.w]}
    node.create_subscription(RobotJointState,'/astribot_chassis/joint_space_states',joint,qos_profile_sensor_data)
    node.create_subscription(Imu,'/livox/imu_front',imu_cb,qos_profile_sensor_data)
    node.create_subscription(TFMessage,'/tf_static',tf_cb,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin(t):
        end=time.monotonic()+t
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
    def call_many(specs,timeout=3):
        jobs=[]
        for kind,name,req in specs:
            client=node.create_client(kind,name);clients.append(client)
            jobs.append((name,client.call_async(req)))
        end=time.monotonic()+timeout
        while any(not f.done() for _,f in jobs) and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        return {name:(f.result() if f.done() else None) for name,f in jobs}
    try:
        spin(3)
        nodes=node.get_node_names_and_namespaces();report['nodes']=[ns.rstrip('/')+'/'+name for name,ns in nodes]
        report['topics']={name:types for name,types in node.get_topic_names_and_types() if any(k in name for k in ('chassis','lidar','imu','tf','robot_description','control_rights'))}
        owners=[]
        for name,ns in nodes:
            if any(s=='/astribot/control_rights' for s,_ in node.get_service_names_and_types_by_node(name,ns)):
                owners.append(ns.rstrip('/')+'/'+name)
        report['sdk_control_service_owners']=owners
        selected=[ns.rstrip('/')+'/'+name for name,ns in nodes if any(k in name.lower() for k in ('supervisor','robot_state','chassis','hardware','lidar','control_driver','yaml_to_rosparam','param_service'))]
        specs=[]
        for name in sorted(set(selected)):
            req=ListParameters.Request();req.depth=0;specs.append((ListParameters,name+'/list_parameters',req))
        listed=call_many(specs);report['parameter_names']={n:(v.result.names if v else 'timeout') for n,v in listed.items()}
        keys={};specs=[]
        for service,response in listed.items():
            if response is None:continue
            names=[n for n in response.result.names if any(k in n.lower() for k in ('chassis','watchdog','timeout','heartbeat','extrinsic','imu','lidar','urdf','robot_description','config','frequency'))
                   and not any(k in n.lower() for k in ('password','token','secret'))]
            if names:
                get=service.rsplit('/',1)[0]+'/get_parameters';keys[get]=names
                req=GetParameters.Request();req.names=names;specs.append((GetParameters,get,req))
        fetched=call_many(specs);report['parameters']={}
        for service,response in fetched.items():
            if response is None:report['parameters'][service]='timeout';continue
            data={}
            for name,value in zip(keys[service],response.values):
                field={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}.get(value.type)
                raw=getattr(value,field) if field else None
                if value.type>=5:raw=list(raw)
                data[name]=raw
            report['parameters'][service]=data
        spin(2)
        report['tf_static']=transforms
        report['chassis_feedback']={'count':len(feedback),'observed_duration_s':feedback[-1]['received_monotonic']-feedback[0]['received_monotonic'] if len(feedback)>1 else 0,
                                    'first':feedback[0] if feedback else None,'last':feedback[-1] if feedback else None,
                                    'max_receipt_gap_s':max((b['received_monotonic']-a['received_monotonic'] for a,b in zip(feedback,feedback[1:])),default=None),
                                    'max_age_s':max((f['age_s'] for f in feedback),default=None),
                                    'non_increasing_stamps':sum(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(feedback,feedback[1:]))}
        report['imu_front']={'count':len(imu),'mean_acceleration_norm_native':sum(math.sqrt(sum(x*x for x in a)) for a in imu)/len(imu) if imu else None}
        output.write_text(json.dumps(report,indent=2,ensure_ascii=False))
        print(json.dumps({k:report[k] for k in ('control_commands_sent','sdk_control_service_owners','chassis_feedback','tf_static','imu_front')},indent=2))
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
