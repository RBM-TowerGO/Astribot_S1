#!/usr/bin/env python3
"""No-hardware regression tests for recording, web notes and export association."""
import io,json,os,sys,tempfile,time,unittest,urllib.request,urllib.error
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import collection_common as c
import mapping_support as m
import export_collection as e
from PIL import Image

class CollectionTests(unittest.TestCase):
 def test_all_cameras_and_required_metadata(self):
  self.assertEqual(len(c.STREAMS),9)
  self.assertEqual(len(c.CAMERA_TOPICS),18)
  for camera,kind in c.STREAMS:self.assertIn('/astribot_camera/%s/%s_image'%(camera,kind),c.REQUIRED)
  self.assertIn('left_wrist_rgbd',c.CAMERAS);self.assertIn('head_stereo',c.CAMERAS)

 def test_event_log_and_failed_log_never_prevents_target_hold(self):
  with tempfile.TemporaryDirectory() as d:
   log=c.EventLog(Path(d)/'events.jsonl');received=[]
   b=c.LoggedBackend(NS(send=lambda p:received.append(p),has_control=lambda:True,close=lambda:None),log)
   b.has_control();b.send([1.,2.,3.]);b.close();log.close()
   rows=[json.loads(x) for x in (Path(d)/'events.jsonl').read_text().splitlines()]
   self.assertEqual([r['event'] for r in rows],['control_rights','chassis_target_sent','control_released'])
   log.error='disk failure';b.send([1.,2.,3.]);self.assertEqual(len(received),2)
   with self.assertRaises(RuntimeError):log.check()
   with self.assertRaises(FileExistsError):c.EventLog(Path(d)/'events.jsonl')

 def test_web_note_token_preview_and_no_motion_route(self):
  with tempfile.TemporaryDirectory() as d:
   path=Path(d);(path/'previews').mkdir();(path/'previews/head_rgbd.jpg').write_bytes(b'jpeg')
   state=m.State('test');state.collection_directory=path;state.notes=c.EventLog(path/'notes.jsonl');state.note_token='secret'
   server=m.web_server(state,0,b'<html>test</html>');base='http://127.0.0.1:%d'%server.server_address[1]
   def post(url,body,token=''):
    return urllib.request.urlopen(urllib.request.Request(base+url,data=json.dumps(body).encode(),headers={'X-Session-Token':token}))
   try:
    with self.assertRaises(urllib.error.HTTPError) as cm:post('/api/note',{'text':'x'})
    self.assertEqual(cm.exception.code,403)
    with post('/api/note',{'kind':'task','text':'去门口'},'secret') as r:self.assertEqual(r.status,200)
    with post('/api/note',{'kind':'note','text':'抵达'},'secret') as r:self.assertEqual(r.status,200)
    with self.assertRaises(urllib.error.HTTPError) as cm:post('/api/move',{},'secret')
    self.assertEqual(cm.exception.code,405)
    self.assertEqual(urllib.request.urlopen(base+'/api/camera/head_rgbd').read(),b'jpeg')
    with self.assertRaises(urllib.error.HTTPError):urllib.request.urlopen(base+'/api/camera/../../task.json')
   finally:server.shutdown();server.server_close();state.notes.close()
   rows=[json.loads(x) for x in (path/'notes.jsonl').read_text().splitlines()]
   self.assertEqual(len(rows),2);self.assertEqual(state.task,'去门口')

 def test_export_nearest_depth_time_tolerance_and_raw_units(self):
  def header(t):return NS(stamp=NS(sec=int(t),nanosec=round((t-int(t))*1e9)),frame_id='camera')
  blob=io.BytesIO();Image.new('RGB',(2,2)).save(blob,format='JPEG')
  color='/astribot_camera/head_rgbd/color_compress/compressed'
  depth='/astribot_camera/head_rgbd/depth_compress'
  meta='/astribot_camera/head_rgbd/depth_image'
  rows=[(color,NS(header=header(10.),data=blob.getvalue()),20_000_000_000),
        (depth,NS(header=header(10.08),data=b'\x01\x00'*4,format='raw'),20_080_000_000),
        (depth,NS(header=header(10.01),data=b'\x02\x00'*4,format='raw'),20_010_000_000),
        (meta,NS(header=header(10.01),image=NS(width=2,height=2,encoding='16UC1',is_bigendian=0,step=4),seq_num=2),20_010_000_000),
        ('/astribot_whole_body/chassis_imu',NS(data=[1.,2.]),20_005_000_000)]
  with tempfile.TemporaryDirectory() as d,patch.object(e,'messages',lambda bag:iter(rows)):
   run=Path(d);(run/'task.json').write_text('{"description":"找门"}')
   result=e.export(run,run/'out',1.,.15)
   self.assertEqual(result['frames'],1)
   frame=json.loads((run/'out/frames.jsonl').read_text())
   self.assertEqual(frame['task'],'找门');self.assertEqual(frame['depth']['delta_ns'],10_000_000)
   self.assertEqual((run/'out'/frame['depth']['file']).read_bytes(),b'\x02\x00'*4)
   self.assertEqual(Image.open(run/'out'/frame['depth']['uint16_png']).getpixel((0,0)),2)
   self.assertIn('LIO odometry',frame['missing'])
   self.assertEqual(frame['context']['/astribot_whole_body/chassis_imu']['time_basis'],'receive')
   with self.assertRaises(FileExistsError):e.export(run,run/'out')

 def test_export_rejects_far_depth_and_retains_failure_report(self):
  blob=io.BytesIO();Image.new('RGB',(2,2)).save(blob,format='JPEG')
  def msg(t,data):return NS(header=NS(stamp=NS(sec=t,nanosec=0)),data=data,format='raw')
  rows=[('/astribot_camera/head_rgbd/color_compress/compressed',msg(10,blob.getvalue()),10_000_000_000),
        ('/astribot_camera/head_rgbd/depth_compress',msg(11,b'00'),11_000_000_000)]
  with tempfile.TemporaryDirectory() as d,patch.object(e,'messages',lambda bag:iter(rows)):
   run=Path(d);e.export(run,run/'out')
   frame=json.loads((run/'out/frames.jsonl').read_text());self.assertIn('depth',frame['missing'])
   rows[0]=(rows[0][0],msg(10,b'broken jpeg'),10_000_000_000)
   with self.assertRaises(Exception):e.export(run,run/'failed')
   report=json.loads((run/'failed/export_report.json').read_text());self.assertTrue(report['errors']);self.assertEqual(report['status'],'incomplete')

 def test_preflight_stale_collection_prevents_driving(self):
  import tg_mapping as t
  with tempfile.TemporaryDirectory() as d:
   state=m.State('test')
   args=NS(replay=None,output_root=d,mapping_only=False)
   session=t.Session(args);session.run.mkdir();session.state=state
   session.procs['sensors']=NS(poll=lambda:None)
   c.atomic_json(session.run/'collection_live.json',dict(updated_monotonic=time.monotonic(),ready=False,error=None,missing=['camera'],stale=[]))
   with self.assertRaisesRegex(RuntimeError,'Collection stale/missing'):session.check(require_streams=True)
   c.atomic_json(session.run/'collection_live.json',dict(updated_monotonic=time.monotonic(),ready=True,error='disk write failed'))
   with self.assertRaisesRegex(RuntimeError,'disk write failed'):session.check()

 def test_camera_cleanup_ignores_only_internal_compressor(self):
  import tg_mapping as t
  observer=t.ROSObserver.__new__(t.ROSObserver);observer.name='observer'
  observer.node=NS(get_subscriptions_info_by_topic=lambda topic:[NS(node_name='compress_camera_driver_node'),NS(node_name='external_viewer')])
  self.assertEqual(observer.other_camera_users(),['external_viewer'])

 def test_teleop_event_file_from_real_dry_run_loop(self):
  import pty,select,subprocess
  with tempfile.TemporaryDirectory() as d:
   master,slave=pty.openpty();path=Path(d)/'teleop.jsonl'
   script=Path(__file__).resolve().parents[1]/'scripts/tg_teleop.py'
   child=subprocess.Popen([sys.executable,str(script),'--dry-run','--event-log',str(path)],stdin=slave,stdout=slave,stderr=slave)
   os.close(slave);output=b''
   try:
    end=time.monotonic()+5
    while b'READY' not in output and time.monotonic()<end:
     if select.select([master],[],[],.1)[0]:output+=os.read(master,65536)
    self.assertIn(b'READY',output)
    os.write(master,b'w');time.sleep(.12);os.write(master,b' ');time.sleep(.1);os.write(master,b'x')
    child.wait(timeout=5);self.assertEqual(child.returncode,0)
   finally:
    if child.poll() is None:child.kill();child.wait()
    os.close(master)
   events=[json.loads(x) for x in path.read_text().splitlines()]
   self.assertTrue(any(x['event']=='chassis_target_sent' for x in events))
   self.assertEqual(events[-1]['event'],'session_end')
   self.assertTrue(any(x['event']=='control_released' for x in events))

if __name__=='__main__':unittest.main(verbosity=2)
