#!/usr/bin/env python3
"""Record an isolated simulation route test through its real web/ROS interface."""
import argparse
import json
import math
from pathlib import Path
import re
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=18766)
    parser.add_argument('--start', nargs=3, type=float, required=True)
    parser.add_argument('--goal', nargs=3, type=float, required=True)
    parser.add_argument('--seconds', type=float, default=900)
    parser.add_argument('--observe-existing', action='store_true',
                        help='Observe a matching running goal without resetting or resubmitting it')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    if args.port == 8766: raise ValueError('Use a separate test port, never the interactive user session')
    output = Path(args.output).resolve()
    if output.exists(): raise ValueError('Choose a new report path')
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    base = 'http://127.0.0.1:'+str(args.port)
    for _ in range(30):
        try: html = opener.open(base, timeout=3).read().decode(); break
        except OSError: time.sleep(1)
    else: raise RuntimeError('Test web unavailable')
    token = re.search("const token='([^']+)'", html).group(1)
    def state(): return json.load(opener.open(base+'/api/state', timeout=5))
    def post(command, pose=()):
        body = json.dumps(dict(zip(('x','y','yaw'), pose))).encode()
        req = urllib.request.Request(base+'/api/'+command, data=body, headers={'X-Session-Token':token})
        opener.open(req, timeout=5).close()
    if state().get('mode') != 'simulation': raise RuntimeError('Not a simulator')
    if args.observe_existing:
        s=state()
        if not s['busy'] or s['start']!=args.start or s['goal']!=args.goal or s['collisions']:
            raise RuntimeError('Existing zero-collision running route does not match requested endpoints')
        before=0; initial=args.start
    else:
        post('start', args.start)
        for _ in range(60):
            time.sleep(.5); s = state()
            if s['initialized'] and s['ready']: break
        else: raise RuntimeError('Initial pose not ready: '+s['status'])
        before = s['collisions']; initial = s['truth']; post('goal', args.goal)
    begin = time.monotonic(); samples = []; maximum = [0.,0.]; cancelled = False
    try:
        while time.monotonic()-begin < args.seconds:
            time.sleep(1); s = state(); elapsed = time.monotonic()-begin
            samples.append(dict(seconds=round(elapsed,3), truth=s['truth'], pose=s['pose'],
                                velocity=s['velocity'], remaining=s['distance_remaining'], status=s['status']))
            maximum = [max(maximum[i],abs(s['velocity'][i])) for i in (0,1)]
            if len(samples)%10 == 1:
                print(json.dumps(dict(seconds=round(elapsed), status=s['status'], truth=s['truth'],
                                      velocity=s['velocity'], remaining=s['distance_remaining']),ensure_ascii=False),flush=True)
            if not s['busy'] and len(samples)>1: break
        else: cancelled=True; post('stop')
    except Exception:
        post('stop'); raise
    report = dict(start=args.start,goal=args.goal,status=s['status'],cancelled_at_time_limit=cancelled,
                  seconds=time.monotonic()-begin, collisions=s['collisions']-before,
                  max_observed_velocity=maximum, displacement_m=math.dist(initial[:2],s['truth'][:2]),
                  diagnostics=s['diagnostics'], samples=samples,
                  passed=not cancelled and s['status']=='已到达目标' and s['collisions']==before
                  and maximum[0]<=.100001 and maximum[1]<=.100001)
    report['position_error_m']=math.dist(s['truth'][:2],args.goal[:2])
    delta=s['truth'][2]-args.goal[2]
    report['yaw_error_deg']=math.degrees(abs(math.atan2(math.sin(delta),math.cos(delta))))
    report['passed']=report['passed'] and report['position_error_m']<=.2 and report['yaw_error_deg']<=10.
    report['observed_existing_goal']=args.observe_existing
    report['truth_trajectory']=s['truth_trajectory']
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','truth_trajectory')},ensure_ascii=False,indent=2),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
