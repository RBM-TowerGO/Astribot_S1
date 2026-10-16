"""Fault isolation tests; fake backend only, never construct ROS or SDK clients."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace as NS, ModuleType
import unittest
from unittest.mock import patch, Mock

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
sys.path.insert(0,str(SCRIPTS))
import tg_mapping as m
import tg_teleop as t
from mapping_support import atomic_json


class IsolationTests(unittest.TestCase):
    def session(self, path):
        obj=m.Session(NS(replay=None,output_root=path,mapping_only=False))
        obj.run.mkdir();obj.manual_active=True
        obj.state.healthy=lambda:True
        return obj

    def test_collection_failure_warns_and_recovers_without_closing_health(self):
        with tempfile.TemporaryDirectory() as d:
            obj=self.session(d);obj.procs['sensors']=NS(poll=lambda:None)
            read,obj.health_write=os.pipe()
            try:
                atomic_json(obj.run/'collection_live.json',dict(updated_monotonic=time.monotonic(),ready=False,error='write failed'))
                obj.check(True);obj.heartbeat()
                self.assertEqual(len(os.read(read,8)),8)
                self.assertIn('collection_error',obj.state.warnings)
                self.assertTrue(obj.report['recording_degraded'])
                atomic_json(obj.run/'collection_live.json',dict(updated_monotonic=time.monotonic(),ready=True,error=None))
                obj.check(True)
                self.assertFalse(obj.state.warnings)
                self.assertFalse(obj.report['warnings']['collection_error']['active'])
                self.assertTrue(obj.report['recording_degraded'])
            finally: obj.revoke();os.close(read)

    def test_slice_compares_same_snapshot_and_stale_preview_is_only_warning(self):
        with tempfile.TemporaryDirectory() as d:
            obj=self.session(d);obj.procs['slice']=NS(poll=lambda:None)
            obj.state.stamps['odom']=20_000_000_000
            atomic_json(obj.run/'slice_live.json',dict(stamp_ns=10_000_000_000,lio_stamp_ns=10_100_000_000,worker_updated=time.time()))
            obj.check(True);self.assertFalse(obj.state.warnings)
            atomic_json(obj.run/'slice_live.json',dict(stamp_ns=10_000_000_000,lio_stamp_ns=13_000_000_000,worker_updated=time.time()-10))
            obj.check(True)
            self.assertEqual(set(obj.state.warnings),{'slice_lag','slice_stale'})
            self.assertFalse(obj.report.get('recording_degraded',False))

    def test_missing_or_corrupt_status_and_dead_workers_keep_manual_control(self):
        with tempfile.TemporaryDirectory() as d:
            obj=self.session(d)
            obj.procs={k:NS(poll=lambda:2) for k in ('sensors','slice','fastlio')}
            obj.check(True)
            (obj.run/'collection_live.json').write_text('{broken')
            obj.check(True)
            self.assertIn('collection_status',obj.state.warnings)
            self.assertIn('process_fastlio',obj.state.warnings)
            self.assertTrue(obj.report['recording_degraded'])

    def test_low_disk_stops_only_recorders_once_and_leaves_teleop(self):
        with tempfile.TemporaryDirectory() as d:
            obj=self.session(d);obj.collect=False
            obj.procs={k:Mock(poll=Mock(return_value=None)) for k in ('teleop','raw_record','lio_record','sensors')}
            with patch.object(m.shutil,'disk_usage',return_value=NS(free=2*1024**3)):
                obj.check();obj.check()
            obj.procs['teleop'].send.assert_not_called()
            for name in ('raw_record','lio_record','sensors'):
                obj.procs[name].send.assert_called_once_with(signal.SIGINT)
            self.assertTrue(obj.report['recording_degraded'])
            obj.capture_stop_at=time.monotonic()-11
            obj.check()
            obj.procs['raw_record'].send.assert_called_with(signal.SIGTERM)
            obj.procs['teleop'].send.assert_not_called()

    def test_heartbeat_continues_while_supervisor_busy_and_eof_still_stops(self):
        with tempfile.TemporaryDirectory() as d:
            obj=self.session(d);read,obj.health_write=os.pipe();os.set_blocking(obj.health_write,False)
            health=t.SessionHealth(read)
            try:
                obj.start_heartbeat();time.sleep(.75)
                health.check(time.monotonic())
                obj.revoke()
                with self.assertRaisesRegex(RuntimeError,'连接关闭'):health.check(time.monotonic())
            finally:obj.revoke();health.close()

    def test_status_output_drops_blocked_writes(self):
        master,slave=pty.openpty();out=t.StatusOutput(slave)
        try:
            with patch.object(t.os,'write',side_effect=BlockingIOError):out.write('status')
            self.assertFalse(os.get_blocking(out.fd))
        finally:out.close();os.close(master);os.close(slave)

    def test_sdk_constructor_restores_handlers_and_empty_error_names_type(self):
        old={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP)}
        api=ModuleType('astribot_sdk.core.astribot_api.astribot_client')
        middleware=ModuleType('astribot_ros_middleware');middleware.spin=lambda:time.sleep(.001)
        class Bot:
            chassis_name='base'
            def __init__(self,**kw):
                for sig in old:signal.signal(sig,signal.SIG_IGN)
                self.astribot_interface=NS(shutdown=lambda:None,have_control_rights=False)
            def get_control_rights_status(self):return True
            def get_current_joints_position(self,*unused):return [[0,0,0]]
            get_desired_joints_position=get_current_joints_position
        api.Astribot=Bot
        with patch.dict(sys.modules,{api.__name__:api,middleware.__name__:middleware}):
            backend=t.SDKBackend()
            try:
                self.assertEqual({s:signal.getsignal(s) for s in old},old)
                backend.error=ValueError()
                with self.assertRaisesRegex(RuntimeError,'ValueError'):backend.check()
            finally:backend.close()

    def test_recording_failure_during_real_keyboard_loop_allows_w_and_x(self):
        code='''
import json,sys
from pathlib import Path
sys.path.insert(0,sys.argv.pop(1))
result=Path(sys.argv.pop(1))
import tg_teleop as t
Original=t.EventLog
class FailedLog(Original):
    def check(self):raise RuntimeError('injected disk failure')
class Probe(t.FakeBackend):
    def close(self):result.write_text(json.dumps(self.p))
t.EventLog=FailedLog;t.FakeBackend=Probe
sys.exit(t.main())
'''
        with tempfile.TemporaryDirectory() as d:
            result=Path(d)/'target.json';master,slave=pty.openpty()
            child=subprocess.Popen([sys.executable,'-c',code,str(SCRIPTS),str(result),'--dry-run'],
                stdin=slave,stdout=slave,stderr=slave,start_new_session=True,env=dict(os.environ,XDG_RUNTIME_DIR=d))
            os.close(slave);output=b''
            try:
                end=time.monotonic()+5
                while b'READY' not in output and time.monotonic()<end:
                    if select.select([master],[],[],.05)[0]:output+=os.read(master,65536)
                self.assertIn(b'READY',output)
                time.sleep(.05);os.write(master,b'w');time.sleep(.12)
                self.assertIsNone(child.poll());os.write(master,b'x')
                self.assertEqual(child.wait(timeout=3),0)
                self.assertGreater(json.loads(result.read_text())[0],0.)
            finally:
                if child.poll() is None:child.kill();child.wait()
                os.close(master)

if __name__=='__main__':unittest.main()
