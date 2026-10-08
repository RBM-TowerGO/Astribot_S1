#!/usr/bin/env python3
"""Isolated adapter test: synthetic messages, unique topics; no SDK or hardware."""
import os
import struct
import subprocess
import time
import uuid
import unittest
ROS_TEST = os.environ.get('ROS_DOMAIN_ID')=='125' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
if ROS_TEST:
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from sensor_msgs.msg import PointCloud2,PointField
    from livox_ros_driver2.msg import CustomMsg
    from std_msgs.msg import Bool
    from ament_index_python.packages import get_package_prefix

@unittest.skipUnless(ROS_TEST, 'Requires isolated Domain 125 ROS environment')

class AdapterROS(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if os.environ.get('ROS_DOMAIN_ID')!='125' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Tests require Domain 125 localhost; never run on robot domain')
        rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        token='dual_test_'+uuid.uuid4().hex[:8];self.node=rclpy.create_node(token);self.out=[];self.fault=[]
        prefix='/'+token
        names=['/livox/lidar_front','/livox/lidar_back','/livox/imu_front','/towergo/lio/lidar','/towergo/lio/imu','/towergo/mapping_fault','/diagnostics']
        remap={t:prefix+'/topic_'+str(i) for i,t in enumerate(names)}
        self.front=self.node.create_publisher(PointCloud2,remap[names[0]],10)
        self.back=self.node.create_publisher(PointCloud2,remap[names[1]],10)
        self.node.create_subscription(CustomMsg,remap[names[3]],self.out.append,10)
        self.node.create_subscription(Bool,remap[names[5]],lambda m:self.fault.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        exe=get_package_prefix('towergo_dual_livox')+'/lib/towergo_dual_livox/dual_adapter'
        cmd=[exe,'--ros-args','-r','__node:='+token+'_adapter','-p','coordinate_contract:=both_raw_topics_already_in_front_lidar_frame','-p','imu_acceleration_unit:=g']
        for a,b in remap.items():cmd+=['-r',a+':='+b]
        self.proc=subprocess.Popen(cmd,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        self.spin_until(lambda:self.front.get_subscription_count()>0 and self.back.get_subscription_count()>0 and self.node.count_publishers(remap[names[3]])>0,5)
        deadline=time.monotonic()+.3
        while time.monotonic()<deadline:rclpy.spin_once(self.node,timeout_sec=.02)
    def tearDown(self):
        self.proc.terminate();self.proc.wait(timeout=5);self.node.destroy_node()
    def spin_until(self,predicate,seconds):
        deadline=time.monotonic()+seconds
        while not predicate() and time.monotonic()<deadline:rclpy.spin_once(self.node,timeout_sec=.02)
        self.assertTrue(predicate())
    def cloud(self,ns,line=0,bad_time=False):
        m=PointCloud2();m.header.frame_id='livox_frame';m.header.stamp.sec=ns//10**9;m.header.stamp.nanosec=ns%10**9
        m.height=1;m.width=11;m.point_step=26;m.row_step=286
        m.fields=[PointField(name=k,offset=off,datatype=typ,count=1) for k,off,typ in [('x',0,7),('y',4,7),('z',8,7),('intensity',12,7),('tag',16,2),('line',17,2),('timestamp',18,8)]]
        m.data=b''.join(struct.pack('<ffffBBd',1.,2.,3.,20.,0,line,float(ns+i*10000000 if not bad_time else ns-10000000)) for i in range(11));return m
    def test_both_sensors_keep_time_and_rear_lines(self):
        self.front.publish(self.cloud(10**9));self.back.publish(self.cloud(10**9));self.spin_until(lambda:bool(self.out),2)
        out=self.out[0];self.assertEqual(out.timebase,10**9);self.assertEqual(out.point_num,20)
        self.assertEqual(sum(p.line==4 for p in out.points),10)
        times=[p.offset_time for p in out.points];self.assertEqual(times,sorted(times));self.assertEqual(times[-1],90000000)
    def test_missing_rear_never_outputs_and_latches_fault(self):
        self.front.publish(self.cloud(10**9));self.spin_until(lambda:bool(self.fault),3)
        self.assertEqual(self.out,[])
        self.back.publish(self.cloud(10**9));time.sleep(.1);rclpy.spin_once(self.node,timeout_sec=.1);self.assertEqual(self.out,[])
    def test_short_rear_gap_resumes_with_dual_windows(self):
        self.front.publish(self.cloud(10**9));self.back.publish(self.cloud(10**9))
        self.spin_until(lambda:bool(self.out),2)
        for stamp in [1110000000,1220000000,1330000000,1440000000]:
            self.front.publish(self.cloud(stamp))
            rclpy.spin_once(self.node,timeout_sec=.02)
        self.back.publish(self.cloud(1440000000))
        self.spin_until(lambda:any(m.timebase==1400000000 for m in self.out),2)
        self.assertFalse(self.fault)
        for m in self.out:
            self.assertTrue(any(p.line<4 for p in m.points))
            self.assertTrue(any(p.line>=4 for p in m.points))
        self.assertFalse(any(m.timebase in [1200000000,1300000000] for m in self.out))

    def test_forward_gap_expires_old_buffer_and_recovers(self):
        self.front.publish(self.cloud(10**9));self.back.publish(self.cloud(10**9))
        self.spin_until(lambda:bool(self.out),2)
        self.front.publish(self.cloud(3*10**9));self.back.publish(self.cloud(3*10**9))
        self.spin_until(lambda:any(m.timebase==3*10**9 for m in self.out),2)
        self.assertFalse(self.fault)

    def test_invalid_rear_line_faults(self):
        self.back.publish(self.cloud(10**9,line=4));self.spin_until(lambda:bool(self.fault),2);self.assertFalse(self.out)
    def test_invalid_rear_timestamp_faults(self):
        self.back.publish(self.cloud(10**9,bad_time=True));self.spin_until(lambda:bool(self.fault),2);self.assertFalse(self.out)
    def test_rear_stall_after_active_fusion_stops_output(self):
        self.front.publish(self.cloud(10**9));self.back.publish(self.cloud(10**9));self.spin_until(lambda:bool(self.out),2)
        count=len(self.out);self.front.publish(self.cloud(1110000000));self.spin_until(lambda:bool(self.fault),3)
        self.assertEqual(len(self.out),count)

if __name__=='__main__':unittest.main(verbosity=2)
