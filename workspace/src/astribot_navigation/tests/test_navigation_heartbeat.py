"""Exercise the real HTTP handler without ROS or a chassis connection."""
import ast
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import urllib.request
import urllib.error

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from real_navigation.core import Lease


def test_stale_browser_snapshot_does_not_report_fault_or_renew_old_task():
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='server')
    ns=dict(Path=Path,__file__=str(ROOT/'scripts/real_navigation/web.py'),
            BaseHTTPRequestHandler=BaseHTTPRequestHandler,ThreadingHTTPServer=ThreadingHTTPServer,
            threading=threading,json=json,time=time,queue=__import__('queue'))
    exec(compile(ast.Module(body=[fn],type_ignores=[]),'web','exec'),ns)
    lease=Lease();owner='a'*32;old=lease.begin(owner,time.monotonic());lease.stop();new=lease.begin(owner,time.monotonic())
    node=SimpleNamespace(token='token',lease=lease,lock=threading.RLock())
    server=ns['server'](node,0)
    opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    def heartbeat(task,who=owner):
        req=urllib.request.Request('http://127.0.0.1:%s/api/heartbeat'%server.server_address[1],
            data=json.dumps(dict(owner=who,task=task)).encode(),
            headers={'X-Session-Token':'token','Content-Type':'application/json'})
        with opener.open(req,timeout=2) as r:return json.load(r)
    try:
        deadline=lease.until
        assert heartbeat(old)=={'superseded':True}
        assert lease.task==new and lease.until==deadline
        assert heartbeat(new)=={'queued':True}
        lease.until=time.monotonic()-1
        try:heartbeat(new)
        except urllib.error.HTTPError as e:assert e.code==400
        else:raise AssertionError('Expired current task must not revive')
        lease.stop();assert heartbeat(new)=={'superseded':True}
        assert not lease.active and lease.task is None
        try:heartbeat(new,'b'*32)
        except urllib.error.HTTPError as e:assert e.code==400
        else:raise AssertionError('Another browser must not obtain control')
    finally:server.shutdown();server.server_close()


def test_transform_tolerance_accepts_recorded_jitter_but_rejects_stale_tf():
    tree=ast.parse((ROOT/'scripts/real_navigation/web.py').read_text())
    expr=next(n.value for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='tf_ok' for t in n.targets) and isinstance(n.value,ast.Compare))
    code=compile(ast.Expression(body=expr),'tf','eval')
    for age,expected in [(.529,True),(.8,True),(.801,False),(-.351,False)]:
        assert eval(code,{'tf_age':age}) is expected
