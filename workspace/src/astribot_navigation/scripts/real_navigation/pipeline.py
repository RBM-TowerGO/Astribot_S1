"""Bounded point-cloud work, independent of ROS and robot control."""
from collections import defaultdict, deque
import threading
from concurrent.futures import ThreadPoolExecutor
import time
import numpy as np


def match_scans(history,now,after=None):
    """Match the same dual scan (20 ms skew), never adjacent 100 ms frames."""
    pairs=[(a,b) for a in history['front'] for b in history['back']
           if abs(a[0]-b[0])<=.02 and all(-.05<=now-v[0]<=.5 for v in (a,b))
           and (after is None or min(a[0],b[0])>after)]
    return max(pairs,key=lambda pair:min(pair[0][0],pair[1][0])) if pairs else None


class Metrics:
    def __init__(self):
        self.lock=threading.Lock();self.samples=defaultdict(lambda:deque(maxlen=2048));self.counts=defaultdict(int)

    def add(self,key,value):
        if np.isfinite(value):
            with self.lock:self.samples[key].append(float(value))

    def count(self,key):
        with self.lock:self.counts[key]+=1

    def snapshot(self):
        with self.lock:
            rows={k:list(v) for k,v in self.samples.items()};counts=dict(self.counts)
        windows={}
        for key,values in rows.items():
            if not values:continue
            samples=np.asarray(values)
            p50,p95=np.percentile(samples,[50,95])
            windows[key]=dict(n=len(values),latest=values[-1],p50=float(p50),p95=float(p95),maximum=float(samples.max()))
        return dict(counters=counts,windows=windows)


class CloudWorker:
    """Two in-flight frames and one waiting/result per sensor.

    Only the ROS thread publishes results. Geometry snapshots are immutable to the
    worker; epoch and source age are checked again by the publishing thread.
    """
    def __init__(self,snapshot,decode,metrics):
        self.snapshot,self.decode,self.metrics=snapshot,decode,metrics
        self.condition=threading.Condition();self.pending={};self.results={}
        self.active=0;self.closed=False;self.error=None;self.completed={}
        self.thread=threading.Thread(target=self.run,name='navigation-cloud',daemon=True);self.thread.start()

    def submit(self,key,msg,epoch):
        with self.condition:
            if key in self.pending:self.metrics.count(key+'_superseded')
            self.pending[key]=(msg,time.monotonic(),epoch)
            self.condition.notify_all()

    def wake(self):
        with self.condition:self.condition.notify_all()

    def take(self):
        with self.condition:
            result=list(self.results.values());self.results.clear()
            error=self.error;self.error=None
            return result,error

    def state(self):
        with self.condition:return dict(pending=len(self.pending),active=self.active,results=len(self.results))

    def close(self):
        with self.condition:self.closed=True;self.condition.notify_all()
        self.thread.join(timeout=2.)

    def run(self):
        # LIO trails cloud endpoints by a scan. Permit a second frame per
        # sensor to wait concurrently, so pose-wait + projection does not
        # reduce a 10 Hz stream to 5 Hz. Never enqueue unbounded pool jobs.
        with ThreadPoolExecutor(max_workers=4,thread_name_prefix='navigation-project') as pool:
            inflight={}
            while True:
                with self.condition:
                    def available():
                        return any(sum(v==key for v in inflight.values())<2 for key in self.pending)
                    self.condition.wait_for(lambda:self.closed or any(f.done() for f in inflight) or available())
                    if self.closed:return
                    for future in list(inflight):
                        if future.done():
                            future.result();del inflight[future]
                    for key in list(self.pending):
                        if sum(v==key for v in inflight.values())>=2:continue
                        item=self.pending.pop(key)
                        future=pool.submit(self.process,key,*item)
                        inflight[future]=key
                        future.add_done_callback(lambda _:self.wake())

    def process(self,key,msg,received,epoch):
        with self.condition:
            if self.closed:return
            self.active+=1
        start=time.monotonic()
        try:
            try:raw,stamps,_=self.decode(msg)
            except ValueError as exc:
                if str(exc)=='Invalid slice absolute point timestamp':
                    self.metrics.count(key+'_invalid_timestamp_frames');return
                raise
            self.metrics.count(key+'_decoded')
            if not len(raw):return
            stamp=float(stamps.max())*1e-9
            while True:
                geometry=self.snapshot()
                if geometry.continuity_epoch!=epoch:
                    self.metrics.count(key+'_epoch_discarded');break
                pose=geometry.at(stamp)
                if pose is not None and geometry.at(float(stamps.min())*1e-9) is not None:
                    begin=time.monotonic()
                    deskewed=geometry.deskew(raw,stamps,pose)
                    if deskewed is not None:
                        points,ranges=geometry.cloud(deskewed,pose,key=='back')
                        self.metrics.add(key+'_processing_ms',(time.monotonic()-begin)*1000)
                        with self.condition:
                            order=(epoch,stamp)
                            if order<=self.completed.get(key,(-1,-float('inf'))):
                                self.metrics.count(key+'_out_of_order_discarded');break
                            self.completed[key]=order
                            if key in self.results:self.metrics.count(key+'_result_superseded')
                            self.results[key]=(key,stamp,points,ranges,epoch,received)
                    break
                with self.condition:
                    if self.closed:return
                    # Keep the active frame long enough for delayed LIO to
                    # bracket it. Replacing it on every new lidar frame
                    # starves output whenever LIO trails by one scan.
                    if time.monotonic()-received>=.3:
                        self.metrics.count(key+'_pose_wait_discarded');break
                    self.condition.wait(timeout=.01)
        except Exception as exc:
            with self.condition:self.error=str(exc)
        finally:
            self.metrics.add(key+'_queue_and_work_ms',(time.monotonic()-received)*1000)
            with self.condition:self.active-=1
