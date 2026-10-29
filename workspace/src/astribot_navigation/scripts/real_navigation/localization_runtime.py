"""ROS-side orchestration. Search/quality run off the Web control callback."""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .localization import MapMatcher, QualityMonitor, Recovery, UpdateExpectation
from .core import SCAN_MAX_SOURCE_AGE
from .confidence import StaticReassessment


class LocalizationRuntime:
    def __init__(self, web):
        self.web=web
        self.matcher=MapMatcher(web.grid,web.sensors.geometry.base_lidar[:2,3])
        self.monitor=QualityMonitor();self.recovery=Recovery()
        self.expectation=UpdateExpectation(web.config.get('amcl_update_min_d',0.),web.config.get('amcl_update_min_a',0.))
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='localization-quality')
        self.future=None;self.search=None;self.generation=0;self.last_scan=None
        self.packet=None;self.last_submit=0.;self.publisher_check=0.;self.publishers={};self.last_nudge=0.;self.nudges=0
        self.log=(web.run/'localization_quality.jsonl').open('a',buffering=1)
        from std_srvs.srv import Empty
        self.empty=Empty
        self.nudge=web.create_client(Empty,'/towergo/nav/request_nomotion_update')
        self.nudge_future=None
        self.reassessment=StaticReassessment();self.previous_amcl_odom=None
        self.amcl_log=(web.run/'amcl_diagnostics.jsonl').open('a',buffering=1)

    def reset(self, cancel_recovery=True):
        self.generation+=1;self.monitor.reset();self.expectation.reset();self.last_scan=None;self.packet=None
        self.nudges=0;self.last_nudge=0.
        self.reassessment.reset();self.previous_amcl_odom=None;self.web.confidence.reset()
        if cancel_recovery:self.recovery.cancel()

    def cancel(self):
        self.generation+=1;self.recovery.cancel();self.recovery.status='canceled';self.reassessment.reset()

    def close(self):
        self.cancel();self.pool.shutdown(wait=True,cancel_futures=True);self.log.close();self.amcl_log.close()

    def compute(self, packet):
        started=time.monotonic()
        packet['metrics']=self.matcher.evaluate(packet['points'],packet['pose'])
        packet['per_sensor']={k:self.matcher.evaluate(v['points'],v['pose']) for k,v in packet['sensors'].items()}
        packet['quality_processing_ms']=(time.monotonic()-started)*1000
        self.log.write(json.dumps(packet,ensure_ascii=False,allow_nan=False)+'\n')
        return packet

    def sample(self, now):
        import rclpy
        from .sensors import FRAME
        from .geometry import scan_points
        w=self.web;s=w.sensors
        if not s.latest_scan:return None
        stamp,cloud=s.latest_scan
        if self.recovery.status=='verifying':
            if w.amcl_stamp<=self.recovery.published_stamp or stamp<self.recovery.published_stamp+.2:return None
        if stamp==self.last_scan:return None
        age=w.get_clock().now().nanoseconds*1e-9-s.latest_scan_source_stamp
        if not -.05<=age<=SCAN_MAX_SOURCE_AGE:return None
        def at(stamp_ns,points):
            t=w.tf.lookup_transform(FRAME['map'],FRAME['base'],rclpy.time.Time(nanoseconds=stamp_ns))
            q=t.transform.rotation;p=t.transform.translation
            rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            yaw=math.atan2(rot[1,0],rot[0,0]);c,sn=math.cos(yaw),math.sin(yaw)
            # Preserve measured roll/pitch in endpoints; search changes planar pose only.
            pts=(np.asarray(points).reshape((-1,3))@rot.T)[:,:2]@np.array([[c,-sn],[sn,c]])
            matrix=np.eye(4);matrix[:3,:3]=rot;matrix[:3,3]=[p.x,p.y,p.z]
            return dict(pose=[p.x,p.y,yaw],points=pts.tolist(),map_from_base=matrix.tolist())
        origin=s.geometry.base_lidar[:3,3]
        fused=at(s.latest_scan_stamp_ns,np.asarray(cloud).reshape((-1,3))+origin)
        sensors={}
        for key,(ts,ranges) in s.scans.items():
            try:sensors[key]=dict(stamp=ts,**at(int(round(ts*1e9)),scan_points(ranges)+origin))
            except Exception:pass
        odom=s.geometry.at(stamp)
        return dict(generation=self.generation,stamp=stamp,source_stamp=s.latest_scan_source_stamp,
                    received_monotonic=now,wall_time=time.time(),source_age_s=age,tf_publishers=self.publishers,
                    stationary=bool(0<=now-w.executor_at<.3 and w.executor_state.get('stationary',False)),
                    odom_from_base=odom.tolist() if odom is not None else None,
                    map_from_odom=(np.asarray(fused['map_from_base'])@np.linalg.inv(odom)).tolist() if odom is not None else None,
                    amcl_stamp=w.amcl_stamp if math.isfinite(w.amcl_stamp) and w.amcl_stamp>0 else None,
                    amcl_covariance=w.amcl_cov if math.isfinite(w.amcl_cov) else None,amcl_confidence=w.confidence.snapshot(),
                    executor=w.executor_state,command=list(w.cmd),sensors=sensors,**fused)

    def tick(self, now):
        w=self.web;r=self.recovery
        if now-self.publisher_check>=1:
            self.publisher_check=now
            self.publishers={t:[dict(name=p.node_name,namespace=p.node_namespace) for p in w.get_publishers_info_by_topic(t)]
                             for t in ('/towergo/nav/tf','/towergo/nav/tf_static','/towergo/nav/amcl_pose')}
        stationary=bool(0<=now-w.executor_at<.3 and w.executor_state.get('stationary',False))
        sensors_ready=w.sensors.healthy()
        r.observe(now,stationary,sensors_ready)
        if self.future is not None and self.future.done():
            try:
                packet=self.future.result()
                if packet['generation']==self.generation and 0<=now-packet['received_monotonic']<.8:
                    self.packet=packet
                    if w.initial_message is not None or r.status=='verifying':self.monitor.invalid=False
                    self.monitor.update(packet['stamp'],packet['received_monotonic'],packet['metrics'],packet['pose'],packet['stationary'])
            except Exception as exc:
                self.monitor.state=dict(status='unavailable',reason='定位质量计算失败：'+str(exc),last_trusted_pose=self.monitor.last_trusted)
                self.monitor.last_received=None
            self.future=None
        if self.monitor.invalid and w.initial_message is None and r.status in ('idle','succeeded'):
            anchor=self.monitor.last_trusted
            if w.lease.active:w.stop('扫描与地图失配，停止任务并等待静止恢复','fault',recoverable=True)
            w.state.update(initialized=False,ready=False,status='定位不可信，等待静止恢复')
            w.localization.update(status='untrusted',reason=self.monitor.state.get('reason'))
            # Failed manual initialization must terminate; no stuck pending request.
            w.initial_message=None
            r.begin(anchor,now)
            w.event('定位不可信：'+str(self.monitor.state.get('reason')))
        if self.search is not None and self.search[1].done():
            generation,future=self.search
            try:result=future.result()
            except Exception as exc:result=dict(ok=False,reason='局部搜索失败：'+str(exc))
            self.search=None
            if r.result(generation,result):
                if not stationary or not sensors_ready or w.lease.active or w.state['busy'] or w.phase!='idle':
                    r.fail('恢复提交前未确认静止或仍有活动任务')
                elif not w.refresh_initialization_transform() or w.localization_reasons():
                    r.fail('恢复提交前扫描变换或 AMCL 未就绪')
                else:self.publish_candidate()
        if self.packet is not None and r.status=='collecting' and w.phase=='idle' and not w.state['busy']:
            if 0<=now-self.packet['received_monotonic']<.8 and self.packet['stationary']:
                if r.add(self.packet['stamp'],self.packet['points']):
                    generation=r.generation;frames=list(r.frames);anchor=list(r.anchor)
                    self.search=(generation,self.pool.submit(self.matcher.search,frames,anchor,r.started+30,
                                                            lambda:r.generation!=generation))
        if self.search is None and self.future is None and now-self.last_submit>=.2:
            try:
                packet=self.sample(now)
                if packet:
                    self.last_scan=packet['stamp'];self.last_submit=now
                    self.future=self.pool.submit(self.compute,packet)
            except Exception:
                pass # Freshness expires if the exact-time transform remains unavailable.
        if r.status=='verifying':
            good=(self.monitor.usable(now) and self.monitor.state.get('status')=='trusted' and
                  w.amcl_stamp>r.published_stamp and self.amcl_is_current(now) and not w.confidence.blocked)
            if r.verify(now,good,w.amcl_stamp):
                w.initial_message=None
                w.localization.update(status='succeeded',reason='定位恢复，请重新提交目标')
                w.state.update(initialized=True,status='定位恢复，请重新提交目标')
                # Executor faults belong to the executor and are never cleared here.
                if not w.executor_state.get('fault'):w.state['error']=None
                w.event('静止局部恢复通过；未恢复导航任务')
        if r.status=='manual_required':
            w.initial_message=None;w.state.update(initialized=False,ready=False,status='定位不可信：'+str(r.reason))
            w.localization.update(status='untrusted',reason=r.reason)
        quality=dict(self.monitor.state)
        if self.monitor.last_received is None or now-self.monitor.last_received>=.8:
            quality.update(status='unavailable',reason='缺少新鲜的扫描时刻地图匹配结果')
        quality['recovery']=r.state()
        w.state['localization_quality']=quality
        w.state['last_trusted_pose']=self.monitor.last_trusted
        # At most three explicit updates per initialization, never a free-running reset.
        if (w.initial_message is not None or r.status=='verifying') and stationary and sensors_ready:
            if self.nudges<3 and now-self.last_nudge>=1 and self.nudge.service_is_ready() and (self.nudge_future is None or self.nudge_future.done()):
                self.nudge_future=self.nudge.call_async(self.empty.Request());self.nudges+=1;self.last_nudge=now
        self.tick_reassessment(now,stationary,sensors_ready)
        return self.monitor.usable(now) and r.status not in ('waiting_stationary','collecting','searching','manual_required')

    def tick_reassessment(self, now, stationary, sensors_ready):
        w=self.web;r=self.recovery
        # Re-evaluate uncertainty only after a confirmed hold; never reset pose or resume a goal.
        confidence=w.confidence
        eligible=(confidence.blocked and confidence.details['valid'] and bool(confidence.details.get('reason'))
                  and w.localization.get('status')=='recovering' and w.initial_message is None
                  and r.status in ('idle','succeeded') and stationary and sensors_ready
                  and not w.executor_state.get('fault') and not w.executor_state.get('holding')
                  and w.executor_state.get('rights') and w.executor_state.get('task') is None
                  and not w.state['busy'] and not w.lease.active and w.phase=='idle'
                  and self.monitor.usable(now) and self.monitor.state.get('status')=='trusted'
                  and w.refresh_initialization_transform() and w.amcl_active and now-w.amcl_lifecycle_at<3)
        due=self.reassessment.due(now,eligible,w.amcl_stamp)
        if due and self.nudge.service_is_ready() and (self.nudge_future is None or self.nudge_future.done()):
            self.nudge_future=self.nudge.call_async(self.empty.Request())
            self.reassessment.sent(now,w.amcl_stamp)
            w.event('协方差静止重评估：第 %d / 3 次' % self.reassessment.count)
        if confidence.status=='trusted' and self.reassessment.status!='manual_required':self.reassessment.reset()
        if self.reassessment.status=='manual_required' and w.localization.get('status')=='recovering':
            w.localization.update(status='manual_required',reason='静止重评估超时，请手动重新选点')
            w.state.update(initialized=False,ready=False,status='静止重评估超时，请手动重新选点')
        w.state['localization_reassessment']=self.reassessment.snapshot()

    def observe_amcl(self, stamp, covariance):
        pose=self.web.sensors.geometry.at(stamp)
        odom=None;delta=None
        if pose is not None:
            odom=[float(pose[0,3]),float(pose[1,3]),math.atan2(pose[1,0],pose[0,0])]
            self.expectation.observe(stamp,odom)
            if self.previous_amcl_odom is not None:
                previous_stamp,previous=self.previous_amcl_odom
                dx,dy=odom[0]-previous[0],odom[1]-previous[1]
                yaw=math.atan2(math.sin(odom[2]-previous[2]),math.cos(odom[2]-previous[2]))
                c,s=math.cos(previous[2]),math.sin(previous[2])
                delta=dict(previous_stamp=previous_stamp,dt=stamp-previous_stamp,
                           x_m=c*dx+s*dy,y_m=-s*dx+c*dy,translation_m=math.hypot(dx,dy),yaw_rad=yaw)
            self.previous_amcl_odom=(stamp,odom)
        else:self.previous_amcl_odom=None
        def finite(v):return float(v) if math.isfinite(v) else None
        w=self.web
        self.amcl_log.write(json.dumps(dict(wall_time=time.time(),received_monotonic=time.monotonic(),
            stamp=stamp,source_age_s=w.get_clock().now().nanoseconds*1e-9-stamp,
            localization_id=w.localization_id,covariance=[finite(v) for v in covariance],
            sigma_x_m=math.sqrt(covariance[0]) if math.isfinite(covariance[0]) and covariance[0]>=0 else None,
            sigma_y_m=math.sqrt(covariance[7]) if math.isfinite(covariance[7]) and covariance[7]>=0 else None,
            sigma_yaw_deg=math.degrees(math.sqrt(covariance[35])) if math.isfinite(covariance[35]) and covariance[35]>=0 else None,
            odom=odom,odom_delta=delta,executor=w.executor_state,command=list(w.cmd)),allow_nan=False)+'\n')

    def amcl_is_current(self, now):
        w=self.web
        if self.expectation.distance<=0 or self.expectation.yaw<=0:
            from .core import localization_fresh
            return localization_fresh(now-w.amcl_at,w.get_clock().now().nanoseconds*1e-9-w.amcl_stamp)
        latest=w.sensors.last_pose
        pose=latest[1] if latest is not None else None
        odom=[pose[0,3],pose[1,3],math.atan2(pose[1,0],pose[0,0])] if pose is not None else None
        return self.expectation.valid(now,odom)

    def publish_candidate(self):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from .sensors import FRAME
        w=self.web;r=self.recovery;pose=r.pose;stamp=w.initial_tf_ns*1e-9
        if not r.published(stamp):return
        self.reset(cancel_recovery=False)
        msg=PoseWithCovarianceStamped();msg.header.frame_id=FRAME['map']
        msg.header.stamp=rclpy.time.Time(nanoseconds=w.initial_tf_ns).to_msg()
        msg.pose.pose.position.x,msg.pose.pose.position.y=pose[:2]
        msg.pose.pose.orientation.z=math.sin(pose[2]/2);msg.pose.pose.orientation.w=math.cos(pose[2]/2)
        msg.pose.covariance[0]=msg.pose.covariance[7]=.15**2;msg.pose.covariance[35]=math.radians(5)**2
        w.initial_stamp=stamp;w.amcl_stamp=stamp;w.amcl_at=-1e20;w.amcl_cov=math.inf
        w.localization_id+=1;w.localization=dict(id=w.localization_id,status='recovering_local',reason='正在验证局部恢复',source_stamp=stamp)
        w.convergence.reset();w.initial.publish(msg)
        w.event('局部恢复候选只发布一次，等待三秒匹配验证')
