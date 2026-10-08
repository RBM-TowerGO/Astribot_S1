"""Dual mapping diagnostics and health checks; no hardware or live ROS."""
import gzip
import urllib.request
import numpy as np
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import dual_lidar_contract as contract
import mapping_support as support
import tg_mapping as mapping

class DualTests(unittest.TestCase):
    def test_observer_close_is_idempotent(self):
        from unittest.mock import Mock
        observer=mapping.ROSObserver.__new__(mapping.ROSObserver)
        observer.rclpy=Mock();observer.thread=Mock();observer.node=Mock()
        observer.close();observer.close()
        observer.rclpy.try_shutdown.assert_called_once_with()
        observer.thread.join.assert_called_once_with(timeout=3)
        observer.node.destroy_node.assert_called_once_with()

    def test_large_preview_compressed_and_precision_bounded(self):
        state=support.State('gzip')
        state.preview.points=np.arange(3000,dtype=np.float32).reshape(-1,3)/333
        server=support.web_server(state,0,b'html')
        try:
            req=urllib.request.Request('http://127.0.0.1:%d/api/state'%server.server_address[1],headers={'Accept-Encoding':'gzip'})
            with urllib.request.urlopen(req) as response:
                self.assertEqual(response.headers['Content-Encoding'],'gzip')
                wire=response.read();raw=gzip.decompress(wire);data=json.loads(raw)
            self.assertLess(len(wire),len(raw)/2)
            self.assertEqual(data['points'][0][1],.003)
        finally: server.shutdown();server.server_close()

    def test_both_lidars_required(self):
        state=support.State('test')
        for key in support.REQUIRED:
            if key!='back': state.seen(key,1,10)
        self.assertFalse(state.ready(10,0))
        state.seen('back',1,10)
        self.assertTrue(state.healthy(10))
        state.seen('front',2,12);self.assertFalse(state.healthy(12.1))

    def test_rear_required_in_bag(self):
        self.assertIn('/livox/lidar_back',support.REQUIRED_RAW_TOPICS)
        self.assertNotIn('/livox/imu_back',support.REQUIRED_RAW_TOPICS)
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'metadata.yaml').write_text('rosbag2_bagfile_information:\n  topics_with_message_count: []\n  relative_file_paths: []\n')
            with self.assertRaises(RuntimeError):support.check_metadata(p,support.REQUIRED_RAW_TOPICS)

    def evidence(self,p,live=False,frame=True):
        cfg=p/'driver.json';cfg.write_text('{}')
        evidence=p/'evidence.json';evidence.write_text(json.dumps({'coordinate_contract':contract.CONTRACT,'frame_convention_verified':frame,'live_alignment_verified':live,'driver_config_sha256':hashlib.sha256(cfg.read_bytes()).hexdigest()}))
        return cfg,evidence

    def test_unverified_alignment_is_advisory(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg,evidence=self.evidence(Path(tmp))
            result=contract.coordinate_diagnostics(cfg,evidence)
            self.assertFalse(result['live_alignment_verified'])
            self.assertFalse(result['startup_restricted_by_calibration'])
            self.assertTrue(result['diagnostic_notes'])
            data=json.loads(evidence.read_text());data['live_alignment_verified']='false';evidence.write_text(json.dumps(data))
            self.assertFalse(contract.coordinate_diagnostics(cfg,evidence)['live_alignment_verified'])

    def test_unknown_frame_is_advisory(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg,evidence=self.evidence(Path(tmp),frame=False)
            result=contract.coordinate_diagnostics(cfg,evidence)
            self.assertFalse(result['startup_restricted_by_calibration'])
            self.assertTrue(any('坐标约定' in n for n in result['diagnostic_notes']))

    def test_changed_driver_config_is_advisory(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg,evidence=self.evidence(Path(tmp),live=True);cfg.write_text('{"changed":true}')
            result=contract.coordinate_diagnostics(cfg,evidence)
            self.assertFalse(result['startup_restricted_by_calibration'])
            self.assertTrue(any('驱动配置' in n for n in result['diagnostic_notes']))

    def test_missing_or_invalid_evidence_is_advisory(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg,evidence=self.evidence(Path(tmp))
            for contents in ['invalid JSON','[]']:
                evidence.write_text(contents)
                self.assertTrue(contract.coordinate_diagnostics(cfg,evidence)['diagnostic_notes'])
            evidence.unlink();cfg.unlink()
            result=contract.coordinate_diagnostics(cfg,evidence)
            self.assertFalse(result['startup_restricted_by_calibration'])
            self.assertFalse(result['live_alignment_verified'])

    def test_verified_fixture_has_no_notes(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg,evidence=self.evidence(Path(tmp),live=True)
            result=contract.coordinate_diagnostics(cfg,evidence)
            self.assertTrue(result['live_alignment_verified'])
            self.assertEqual(result['diagnostic_notes'],[])

    def test_fastlio_eight_lines_features_disabled(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'base.yaml';p.write_text('/**:\n  ros__parameters:\n    common: {}\n    mapping: {}\n    pcd_save: {}\n    publish: {}\n')
            params=mapping.parameters(p,mapping.topic_names('/dual'),True)['/**']['ros__parameters']
            self.assertEqual(params['preprocess']['scan_line'],8)
            self.assertFalse(params['feature_extract_enable'])
            self.assertFalse(params['mapping']['extrinsic_est_en'])

    def test_web_stats_bounded_and_visible(self):
        state=support.State('test');state.fusion={'front_input_points':42,'back_input_points':38,'windows':1}
        state.validation_warning='unverified'
        out=state.snapshot();self.assertEqual(out['mode'],'dual_lidar');self.assertEqual(out['fusion']['back_input_points'],38)
        self.assertEqual(out['validation_warning'],'unverified')

    def test_rear_fault_revokes_health_before_child_cleanup(self):
        with tempfile.TemporaryDirectory() as tmp:
            obj=mapping.Session(NS(replay='unused',output_root=tmp,port=0));obj.run.mkdir()
            reader,obj.health_write=os.pipe();events=[]
            def stop(timeout):
                events.append(os.read(reader,8))
            obj.procs={'teleop':NS(stop=stop)};obj.state.error='back: time reset'
            try:
                with self.assertRaisesRegex(RuntimeError,'back'):obj.check()
                obj.cleanup();self.assertEqual(events,[b''])
            finally:os.close(reader)

if __name__=='__main__':unittest.main(verbosity=2)
