#!/usr/bin/env python3
"""Inject a wrong estimate ONLY into localhost Domain 125 mock replay.
Start tg-navigation.sh --mode mock --replay BAG --port 8769 first.
"""
import argparse,json,math,os,re,time,urllib.request
from pathlib import Path

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);args=ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='125' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':raise RuntimeError('Requires isolated Domain 125')
    url='http://127.0.0.1:8769'
    def state():
        s=json.load(urllib.request.urlopen(url+'/api/state',timeout=2))
        if s['mode']!='mock' or not s['replay']:raise RuntimeError('Refusing to inject into non-mock session')
        return s
    deadline=time.monotonic()+40
    while time.monotonic()<deadline:
        try:
            s=state()
            if s.get('sensors_ready') and s.get('initialization_tf_age') is not None and s['initialization_tf_age']<.5:break
        except OSError:pass
        time.sleep(.3)
    else:raise RuntimeError('isolated sensors unavailable')
    token=re.search("token='([^']+)'",urllib.request.urlopen(url).read().decode())[1]
    request=urllib.request.Request(url+'/api/localize',data=json.dumps(dict(owner='isolated-localization-validation',x=.0012,y=-.2415,yaw=.7991)).encode(),headers={'X-Session-Token':token,'Content-Type':'application/json'})
    print(urllib.request.urlopen(request).read().decode(),flush=True)
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        s=state()
        if s['initialized']:break
        time.sleep(.3)
    else:raise RuntimeError('initialization failed')
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);n=rclpy.create_node('isolated_estimate_fault_injection');pub=n.create_publisher(PoseWithCovarianceStamped,'/towergo/nav/initialpose',10)
    try:
        deadline=time.monotonic()+10
        while time.monotonic()<deadline and pub.get_subscription_count()<1:rclpy.spin_once(n,timeout_sec=.1)
        if pub.get_subscription_count()!=1:raise RuntimeError('Expected exactly one isolated AMCL subscriber')
        p=state()['pose'];m=PoseWithCovarianceStamped();m.header.frame_id='tg_nav_map';m.header.stamp=n.get_clock().now().to_msg()
        m.pose.pose.position.x=p[0]+.6;m.pose.pose.position.y=p[1]-.4;y=p[2]+.25
        m.pose.pose.orientation.z=math.sin(y/2);m.pose.pose.orientation.w=math.cos(y/2)
        m.pose.covariance[0]=m.pose.covariance[7]=.0001;m.pose.covariance[35]=.0001;pub.publish(m)
        deadline=time.monotonic()+38;rows=[];last=None
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.1);s=state();r=s['localization_quality']['recovery']
            rows.append({k:s.get(k) for k in ['status','pose','localization_quality','ready_checks','busy','command_velocity']})
            if r['status']!=last:print('recovery',r,flush=True);last=r['status']
            if r['status'] in ('succeeded','manual_required'):break
            time.sleep(.2)
        Path(args.output).write_text(json.dumps(rows,ensure_ascii=False,indent=2))
        print('FINAL',s['status'],flush=True)
        if any(x['busy'] or x['command_velocity']!=[0.,0.] for x in rows):raise RuntimeError('Unexpected motion task during localization-only experiment')
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
