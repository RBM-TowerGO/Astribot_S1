#!/usr/bin/env python3
"""Exercise replay-only web/ROS control handshake. Refuses live and normal UI ports."""
import argparse
import json
import re
import time
import urllib.error
import urllib.request
from pathlib import Path


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--port',type=int,default=18767)
    p.add_argument('--initial',type=float,nargs=3,required=True)
    p.add_argument('--goal',type=float,nargs=3,required=True)
    p.add_argument('--speed',type=float,default=.001)
    p.add_argument('--output',required=True)
    args=p.parse_args()
    if args.port in (8765,8766,8767):raise ValueError('Use an independent validation port')
    output=Path(args.output)
    if output.exists():raise ValueError('Report already exists')
    base='http://127.0.0.1:'+str(args.port)
    def state():return json.load(urllib.request.urlopen(base+'/api/state',timeout=3))
    d=state()
    if not d.get('replay') or d.get('mode')!='mock':raise ValueError('Only replay + mock executor may be tested')
    html=urllib.request.urlopen(base,timeout=3).read().decode()
    token=re.search("token='([^']+)'",html).group(1)
    import secrets
    owner=secrets.token_hex(16)
    def post(name,obj):
        return urllib.request.urlopen(urllib.request.Request(base+'/api/'+name,
            data=json.dumps(dict(obj,owner=owner)).encode(),headers={'X-Session-Token':token,'Content-Type':'application/json'}),timeout=3).status
    def pose(v):return dict(zip(('x','y','yaw'),v))
    report={'kind':'replay_interface_validation','hardware_commands_sent':False,'route_success_claimed':False,'samples':[]}
    try:
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            snapshot=state()
            if 'scan' in snapshot.get('sensor_health',{}) and max(snapshot['sensor_health'].values())<.5:break
            time.sleep(.1)
        post('localize',pose(args.initial))
        deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            d=state()
            if d['ready'] and d['initialized']:break
            time.sleep(.1)
        if not d['ready'] or not d['initialized']:raise RuntimeError('Localization/Nav2 not ready: '+str(d.get('error'))+' '+str(d.get('ready_checks')))
        report['localization_ready']=True
        try:
            post('goal',dict(pose(args.goal),speed=0))
            raise RuntimeError('Zero speed incorrectly accepted')
        except urllib.error.HTTPError as exc:
            if exc.code!=400:raise
        report['zero_rejected']=True
        post('goal',dict(pose(args.goal),speed=args.speed))
        deadline=time.monotonic()+6;accepted=False;executor_limit=False
        while time.monotonic()<deadline:
            d=state()
            report['samples'].append({k:d.get(k) for k in ('task','phase','status','error','executor','requested_speed')})
            executor_limit |= d['executor'].get('speed')==args.speed
            accepted |= d['phase']=='running'
            if d['task'] and d['busy'] and d['phase'] in ('preparing','accepting','running'):
                try:post('heartbeat',{'task':d['task']})
                except urllib.error.HTTPError:pass
            if accepted and d['phase']=='stopping':break
            time.sleep(.05)
        report.update(goal_accepted=accepted,executor_limit_confirmed=executor_limit)
        if not executor_limit or not accepted:raise RuntimeError('Goal/limit handshake failed')
        post('stop',{})
        time.sleep(.3)
        d=state()
        report['executor_revoked']=d['executor'].get('task') is None and d['executor'].get('command')==[0.,0.]
        if not report['executor_revoked']:raise RuntimeError('Executor permit not revoked')
        report['passed']=True
    except Exception as exc:
        report.update(passed=False,error=str(exc))
        raise
    finally:
        try:post('stop',{})
        except Exception as exc:report['cleanup_error']=str(exc)
        output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='samples'},ensure_ascii=False))

if __name__=='__main__':main()
