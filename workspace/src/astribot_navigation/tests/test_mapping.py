#!/usr/bin/env python3
"""Software-only mapping tests. No SDK, sensor service or live ROS node."""
import csv
import json
import os
from pathlib import Path
import pty
import signal
import struct
import subprocess
import sys
import tempfile
import termios
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch, Mock
import urllib.request
import urllib.error

import numpy as np
SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
import mapping_support as s
import mapping_finalize as f
import tg_mapping as m
import tg_teleop as t


class StateTests(unittest.TestCase):
    def test_readiness_needs_ten_seconds_and_stable_streams(self):
        state=s.State('test')
        self.assertFalse(state.ready(0,0))
        for i in range(1,12):
            for key in s.REQUIRED: state.seen(key,i,i)
            self.assertEqual(state.ready(i,0),i>=10)
        self.assertFalse(state.ready(14,0))
        for key in s.REQUIRED: state.seen(key,15,15)
        self.assertFalse(state.ready(15,0))

    def test_backward_time_fault(self):
        state=s.State('test'); state.seen('front',10,1); state.seen('front',9,2)
        self.assertIn('timestamp',state.error)
        self.assertFalse(state.healthy(2))

    def test_preview_bounded_and_includes_latest_pose(self):
        preview=s.Preview(max_points=100,max_path=10)
        rng=np.random.default_rng(2)
        for i in range(30):
            preview.cloud(rng.normal(size=(300,3))*i)
            preview.odom([i,0,0],[0,0,0,1])
            self.assertLessEqual(len(preview.points),100)
            self.assertLessEqual(len(preview.path),10)
        self.assertEqual(preview.path[-1],[29,0,0])
        self.assertGreater(preview.voxel,.1)

    def test_cloud_layout_padding_and_endianness(self):
        fields=[NS(name=k,datatype=7,count=1,offset=i*4) for i,k in enumerate('xyz')]
        for big in [False,True]:
            data=bytearray(40)
            data[:12]=struct.pack('>fff' if big else '<fff',1,2,3)
            data[20:32]=struct.pack('>fff' if big else '<fff',4,5,6)
            msg=NS(fields=fields,width=1,height=2,point_step=16,row_step=20,data=data,is_bigendian=big)
            np.testing.assert_equal(s.cloud_xyz(msg),[[1,2,3],[4,5,6]])
            msg.data=b'x'
            with self.assertRaises(ValueError): s.cloud_xyz(msg)

    def test_read_only_http_and_port_conflict(self):
        state=s.State('test'); server=s.web_server(state,0,b'<title>test</title>')
        port=server.server_address[1]
        try:
            self.assertEqual(server.server_address[0],'127.0.0.1')
            snapshot=json.load(urllib.request.urlopen('http://127.0.0.1:%d/api/state'%port))
            self.assertEqual(snapshot['run_id'],'test')
            with self.assertRaises(urllib.error.HTTPError) as cm:
                urllib.request.urlopen(urllib.request.Request('http://127.0.0.1:%d/'%port,data=b'x'))
            self.assertEqual(cm.exception.code,405)
            with self.assertRaises(OSError): s.web_server(state,port,b'')
        finally: server.shutdown();server.server_close()


class ExportTests(unittest.TestCase):
    def test_full_voxel_indices_not_old_xor_hash(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=f.VoxelStore(Path(tmp)/'a.sqlite',resolution=1)
            try:
                # These two indices collided under the previous XOR key.
                store.add(np.array([[0,0,0],[19349663,73856093,0],[.1,.1,.1]],float))
                self.assertEqual(store.count(),2)
            finally: store.close()

    def test_pcd_and_png(self):
        from PIL import Image
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp); store=f.VoxelStore(d/'v.sqlite')
            try:
                store.add(np.array([[0,0,0],[1,2,3],[1,2,3]],float))
                (d/'trajectory.csv').write_text('stamp_ns,x,y,z,qx,qy,qz,qw\n1,0,0,0,0,0,0,1\n2,1,2,3,0,0,0,1\n')
                self.assertEqual(f.products(store,d/'trajectory.csv',d),2)
                raw=(d/'map.pcd').read_bytes()
                self.assertIn(b'POINTS 2',raw)
                self.assertEqual(len(raw.split(b'DATA binary\n')[1]),24)
                with Image.open(d/'preview.png') as image: self.assertEqual(image.size,(1200,1200))
            finally: store.close()

    def test_export_interruption_keeps_recorded_input(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);store=f.VoxelStore(d/'v.sqlite')
            try:
                store.add(np.array([[0,0,0]],float))
                (d/'trajectory.csv').write_text('stamp_ns,x,y,z\n1,0,0,0\n')
                with self.assertRaises(InterruptedError):
                    f.products(store,d/'trajectory.csv',d,cancelled=lambda:True)
                self.assertEqual(store.count(),1)
            finally: store.close()

    def test_metadata_rejects_missing_topics(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp)
            (d/'metadata.yaml').write_text('rosbag2_bagfile_information:\n  topics_with_message_count: []\n  relative_file_paths: []\n')
            with self.assertRaises(RuntimeError): s.check_metadata(d,['/cloud'])

    def test_finalize_refuses_existing_results_before_ros_import(self):
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);(d/'session.json').write_text(json.dumps({'topics':{'odom':'o','cloud':'c'}}))
            (d/'map.pcd').write_text('existing')
            with patch.object(f,'check_metadata'):
                with self.assertRaisesRegex(RuntimeError,'already exists'): f.finalize(d)
            self.assertEqual((d/'map.pcd').read_text(),'existing')


class SessionTests(unittest.TestCase):
    def session(self,tmp):
        args=NS(replay='/unused',output_root=tmp,port=0,linear_speed=.05,angular_speed=.1,replay_seconds=0)
        obj=m.Session(args);obj.run.mkdir();return obj

    def test_low_disk_and_process_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            obj=self.session(tmp)
            with patch.object(m.shutil,'disk_usage',return_value=NS(free=2*1024**3)):
                with self.assertRaisesRegex(RuntimeError,'3 GiB'): obj.check()
            obj.procs={'fastlio':NS(poll=lambda:1)}
            with self.assertRaisesRegex(RuntimeError,'fastlio exited'): obj.check()

    def test_replay_natural_eof_race_is_not_a_fault(self):
        with tempfile.TemporaryDirectory() as tmp:
            obj=self.session(tmp)
            obj.procs={'player':NS(poll=Mock(side_effect=[None,0,0]))}
            with patch.object(obj.state,'healthy',return_value=True),patch.object(m.time,'sleep'):
                obj.drive()
            self.assertIsNone(obj.report['error'])

    def test_sensor_failure_revokes_health_and_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            obj=self.session(tmp); read,obj.health_write=os.pipe();order=[]
            obj.procs={k:NS(stop=lambda timeout,k=k: order.append(k))
                       for k in ['raw_record','adapter','teleop','fastlio','lio_record']}
            obj.cleanup()
            self.assertEqual(os.read(read,8),b'');os.close(read)
            self.assertEqual(order,['teleop','fastlio','adapter','lio_record','raw_record'])

    def test_cleanup_only_owned_lidar_and_preserves_other_users(self):
        with tempfile.TemporaryDirectory() as tmp:
            obj=self.session(tmp);obj.observer=Mock()
            obj.cleanup();obj.observer.lidar.assert_not_called()
            obj.owned_lidar=True;obj.observer.other_sensor_users.return_value=['another_recorder']
            with patch.object(m.time,'sleep'): obj.cleanup()
            obj.observer.lidar.assert_not_called()
            self.assertEqual(obj.report['lidar_left_for_users'],['another_recorder'])
            obj.observer.other_sensor_users.return_value=[]
            with patch.object(m.time,'sleep'): obj.cleanup()
            obj.observer.lidar.assert_called_once_with(False)

    def test_failed_initialization_never_starts_teleop(self):
        with tempfile.TemporaryDirectory() as tmp:
            obj=self.session(tmp)
            with patch.object(obj,'prepare'),patch.object(obj,'start_mapping',side_effect=RuntimeError('not ready')), \
                 patch.object(obj,'drive') as drive,patch.object(m,'check_metadata',return_value={}):
                self.assertEqual(obj.execute(),2)
            drive.assert_not_called();self.assertEqual(obj.report['status'],'incomplete')

    def test_parameters_isolate_and_disable_unbounded_pcd(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp)/'config.yaml'
            base.write_text('/**:\n  ros__parameters:\n    common: {}\n    mapping: {}\n    pcd_save: {}\n    publish: {}\n')
            topics=m.topic_names('/towergo/mapping/test')
            p=m.parameters(base,topics,True)['/**']['ros__parameters']
            self.assertTrue(p['use_sim_time']);self.assertFalse(p['pcd_save']['pcd_save_en'])
            self.assertFalse(p['mapping']['extrinsic_est_en'])
            self.assertEqual(p['common']['lid_topic'],topics['lidar'])

    def test_live_lidar_reuse_and_failed_start(self):
        # Guard the acquisition decision without executing service calls/processes.
        with tempfile.TemporaryDirectory() as tmp:
            obj=self.session(tmp);obj.replay=False;obj.observer=Mock()
            obj.observer.node.count_publishers.return_value=1
            with patch.object(obj,'pause'),patch.object(obj,'spawn',side_effect=RuntimeError('stop test before process start')):
                with self.assertRaisesRegex(RuntimeError,'stop test'): obj.start_mapping()
            obj.observer.lidar.assert_not_called();self.assertFalse(obj.owned_lidar)
            obj.observer.node.count_publishers.return_value=0
            obj.observer.lidar.side_effect=RuntimeError('service rejected')
            with patch.object(obj,'pause'),patch.object(obj,'spawn') as spawn:
                with self.assertRaisesRegex(RuntimeError,'service rejected'): obj.start_mapping()
            spawn.assert_not_called();self.assertFalse(obj.owned_lidar)

    def test_owned_process_does_not_signal_unrelated_process(self):
        unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'])
        with tempfile.TemporaryDirectory() as tmp:
            owned=s.OwnedProcess('test',[sys.executable,'-c','import time; time.sleep(30)'],Path(tmp)/'log')
            try:
                owned.stop();self.assertIsNone(unrelated.poll())
            finally:
                unrelated.terminate();unrelated.wait()


class HeartbeatTests(unittest.TestCase):
    def test_stale_missing_eof_and_timestamp(self):
        for case in ['missing','old','eof','nan','future']:
            read,write=os.pipe(); health=t.SessionHealth(read)
            try:
                if case=='old': os.write(write,struct.pack('!d',9))
                if case=='nan': os.write(write,struct.pack('!d',float('nan')))
                if case=='future': os.write(write,struct.pack('!d',20))
                if case=='eof': os.close(write);write=None
                with self.assertRaises(RuntimeError): health.check(10)
            finally:
                health.close()
                if write is not None: os.close(write)
        read,write=os.pipe();health=t.SessionHealth(read)
        try:
            os.write(write,struct.pack('!d',10));health.check(10.1)
        finally: os.close(write);health.close()

    def test_pty_supervisor_disconnect_exits_and_restores(self):
        with tempfile.TemporaryDirectory() as tmp:
            master,slave=pty.openpty();saved=termios.tcgetattr(slave)
            read,write=os.pipe();os.write(write,struct.pack('!d',time.monotonic()))
            p=subprocess.Popen([str(SCRIPTS/'tg-teleop.sh'),'--dry-run','--session-fd',str(read)],
                               stdin=slave,stdout=slave,stderr=slave,pass_fds=(read,),start_new_session=True,
                               env=dict(os.environ,XDG_RUNTIME_DIR=tmp))
            os.close(read)
            try:
                for _ in range(4):
                    time.sleep(.08);os.write(write,struct.pack('!d',time.monotonic()))
                os.write(master,b'w');time.sleep(.03);os.close(write);write=None
                self.assertEqual(p.wait(timeout=3),2)
                self.assertEqual(termios.tcgetattr(slave),saved)
            finally:
                if write is not None: os.close(write)
                if p.poll() is None: p.kill();p.wait()
                os.close(master);os.close(slave)


if __name__=='__main__': unittest.main(verbosity=2)
