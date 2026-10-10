"""Reproducible offline P1 benchmark with an external process watchdog."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import threading
import time
import traceback
import psutil
from inference_core import ROOT, read_config


def save(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')


def percentile(values, q):
    a = sorted(values)
    pos = (len(a)-1)*q
    lo = int(pos)
    hi = min(lo+1,len(a)-1)
    return a[lo]+(a[hi]-a[lo])*(pos-lo)


class Worker:
    def __init__(self, directory, name, startup_timeout, command=None):
        self.name = name
        self.phase = 'startup'
        self.stop_sampling = threading.Event()
        self.samples = []
        self.err = (directory/(name+'-stderr.log')).open('w')
        self.resource_file = (directory/(name+'-resources.jsonl')).open('w')
        self.started = time.perf_counter()
        self.proc = subprocess.Popen(command or [sys.executable,'-u',str(ROOT/'scripts/benchmark_worker.py')],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.err,
            text=True, bufsize=1, start_new_session=True, cwd=ROOT)
        self.thread = threading.Thread(target=self.sample, daemon=True)
        self.thread.start()
        try:
            self.ready = self.receive(startup_timeout)
            if self.ready.get('event') != 'ready':
                raise RuntimeError(self.ready)
            self.startup_seconds = time.perf_counter()-self.started
        except BaseException:
            self.close()
            raise

    def sample(self):
        process = psutil.Process(self.proc.pid)
        process.cpu_percent()
        psutil.cpu_percent()
        while not self.stop_sampling.is_set():
            v, s = psutil.virtual_memory(), psutil.swap_memory()
            try:
                rss = process.memory_info().rss
                cpu = process.cpu_percent()
            except psutil.Error:
                rss, cpu = None, None
            row = dict(elapsed_seconds=time.perf_counter()-self.started, phase=self.phase,
                       system_total_bytes=v.total, system_available_bytes=v.available,
                       system_total_minus_available_bytes=v.total-v.available,
                       swap_used_bytes=s.used, process_rss_bytes=rss, process_cpu_percent=cpu,
                       system_cpu_percent=psutil.cpu_percent(), load_average=list(os.getloadavg()))
            self.samples.append(row)
            self.resource_file.write(json.dumps(row)+'\n')
            self.resource_file.flush()
            self.stop_sampling.wait(0.1)

    def receive(self, timeout):
        with selectors.DefaultSelector() as selector:
            selector.register(self.proc.stdout, selectors.EVENT_READ)
            if not selector.select(timeout):
                raise TimeoutError(f'{self.name}/{self.phase} exceeded {timeout}s')
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError(f'{self.name}: worker exited unexpectedly, code={self.proc.poll()}')
        event = json.loads(line)
        if event.get('event') == 'fatal':
            raise RuntimeError(event)
        return event

    def request(self, command, timeout):
        started = time.perf_counter()
        self.proc.stdin.write(json.dumps(command)+'\n')
        self.proc.stdin.flush()
        event = self.receive(timeout)
        if event.get('event') != 'result':
            raise RuntimeError(event)
        result = event['result']
        result['controller_roundtrip_seconds'] = time.perf_counter()-started
        return result

    def close(self):
        if self.proc.poll() is None:
            os.killpg(self.proc.pid, signal.SIGTERM)
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.proc.pid, signal.SIGKILL)
                self.proc.wait(timeout=5)
        self.stop_sampling.set()
        self.thread.join(timeout=2)
        self.resource_file.close()
        self.err.close()
        self.proc.stdin.close()
        self.proc.stdout.close()


def resource_summary(rows):
    if not rows:
        return {}
    rss = [r['process_rss_bytes'] for r in rows if r['process_rss_bytes'] is not None]
    return dict(samples=len(rows), nominal_interval_seconds=0.1,
                max_observed_interval_seconds=max([b['elapsed_seconds']-a['elapsed_seconds'] for a,b in zip(rows,rows[1:])] or [0]),
                process_rss_peak_bytes=max(rss,default=0),
                system_available_min_bytes=min(r['system_available_bytes'] for r in rows),
                system_total_minus_available_peak_bytes=max(r['system_total_minus_available_bytes'] for r in rows),
                swap_used_peak_bytes=max(r['swap_used_bytes'] for r in rows))


def run_check(directory, name, command):
    with (directory/(name+'.log')).open('w') as log:
        result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, timeout=300)
    if result.returncode:
        raise RuntimeError(f'{name} failed with {result.returncode}; see log')
    return dict(passed=True, log=name+'.log')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir')
    parser.add_argument('--warmup',type=int,default=5)
    parser.add_argument('--requests',type=int,default=100)
    parser.add_argument('--startup-runs',type=int,default=3)
    parser.add_argument('--request-timeout',type=float,default=120)
    parser.add_argument('--startup-timeout',type=float,default=300)
    args = parser.parse_args()
    if min(args.requests,args.startup_runs,args.request_timeout,args.startup_timeout)<=0 or args.warmup<0:
        parser.error('Counts and timeouts must be positive; warmup may be zero')
    directory = Path(args.output_dir) if args.output_dir else ROOT/'reports'/('p1-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    if not directory.is_absolute():
        directory = ROOT/directory
    directory.mkdir(parents=True,exist_ok=False)
    os.environ.update(HF_HUB_OFFLINE='1',TRANSFORMERS_OFFLINE='1',TOKENIZERS_PARALLELISM='false')
    report = dict(timestamp_utc=datetime.now(timezone.utc).isoformat(), status='running',
                  scope='FP16 synthetic offline deployment baseline; no navigation accuracy claim',
                  startup_definition='fresh process; OS cache uncontrolled; no reboot or cache eviction',
                  args=vars(args),config=read_config(),startup=[],warmup=[],requests=[],failures=[],checks={},resources={})
    paths = list((ROOT/'scripts').glob('*.py'))+[ROOT/'configs/inference.json']+list((ROOT/'assets/samples').glob('*.png'))
    report['sha256'] = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    for key,cmd in [('git_head',['git','rev-parse','HEAD']),('git_status',['git','status','--short']),
                    ('branch',['git','branch','--show-current']),('power_mode',['/usr/sbin/nvpmodel','-q']),
                    ('background_processes',['ps','-eo','pid,comm,%cpu,%mem','--sort=-%cpu']),
                    ('pip_freeze',[sys.executable,'-m','pip','freeze'])]:
        r = subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True)
        report[key] = dict(returncode=r.returncode,stdout=r.stdout,stderr=r.stderr)
    save(directory/'report.json',report)
    print(f'REPORT_DIR={directory}',flush=True)
    active = None
    raw_file = (directory/'requests.jsonl').open('w')

    def record(phase,index,result):
        row = dict(phase=phase,index=index,**result)
        raw_file.write(json.dumps(row,ensure_ascii=False)+'\n')
        raw_file.flush()
        report[phase].append(row)
        save(directory/'report.json',report)

    try:
        for name,cmd in [
            ('pip-check',[sys.executable,'-m','pip','check']),
            ('contract-tests',[sys.executable,str(ROOT/'scripts/test_inference_contract.py'),'-v']),
            ('environment',[sys.executable,str(ROOT/'scripts/check_environment.py'),'--output',str(directory/'environment-check.json')]),
            ('smoke',[sys.executable,str(ROOT/'scripts/smoke_inference.py'),'--output',str(directory/'smoke-inference.json')])]:
            print(f'CHECK {name}',flush=True)
            report['checks'][name] = run_check(directory,name,cmd)
        for index in range(args.startup_runs):
            name = f'startup-{index+1}'
            print(name,flush=True)
            active = Worker(directory,name,args.startup_timeout)
            active.phase = 'first_request'
            expected = ('A','B')[index%2]
            result = active.request(dict(sample=f'assets/samples/red_{expected}.png',expected=expected),args.request_timeout)
            result.update(process_start_to_ready_seconds=active.startup_seconds,
                          process_start_to_first_result_seconds=time.perf_counter()-active.started,
                          load_seconds=active.ready['load_seconds'],load_memory=active.ready)
            record('startup',index,result)
            active.close()
            report['resources'][name] = resource_summary(active.samples)
            active = None
            if not result['passed']:
                raise RuntimeError('Startup inference failed')
        print('HOT RUN',flush=True)
        active = Worker(directory,'hot',args.startup_timeout)
        report['hot_load'] = dict(process_start_to_ready_seconds=active.startup_seconds,**active.ready)
        for phase,count in [('warmup',args.warmup),('requests',args.requests)]:
            active.phase = phase
            for index in range(count):
                expected = ('A','B')[index%2]
                result = active.request(dict(sample=f'assets/samples/red_{expected}.png',expected=expected),args.request_timeout)
                record(phase,index,result)
                if (index+1)%10 == 0:
                    print(f'{phase} {index+1}/{count}',flush=True)
        active.phase = 'failure_tests'
        for name,overrides,error in [('truncation',dict(max_new_tokens=1),'truncated_or_missing_eos'),
                                     ('input-limit',dict(max_input_tokens=1),'input_limit_exceeded')]:
            result = active.request(dict(sample='assets/samples/red_A.png',expected='A',**overrides),args.request_timeout)
            report['checks'][name] = dict(passed=not result['passed'] and error in result['errors'],result=result)
            save(directory/'report.json',report)
        active.close()
        report['resources']['hot'] = resource_summary(active.samples)
        report['resources']['hot_formal'] = resource_summary([r for r in active.samples if r['phase']=='requests'])
        active = None
        # Deliberately stalled CPU-only child; uses the same production watchdog and cleanup.
        hang_code = 'import json,sys,time; print(json.dumps({"event":"ready"}),flush=True); sys.stdin.readline(); time.sleep(300)'
        active = Worker(directory,'timeout-fixture',10,[sys.executable,'-u','-c',hang_code])
        active.phase = 'timeout_fixture'
        timed_out = False
        try:
            active.request({},0.3)
        except TimeoutError as exc:
            timed_out = True
            save(directory/'timeout-failure.json',dict(error='request_timeout',detail=str(exc),expected=True))
        active.close()
        report['checks']['timeout'] = dict(passed=timed_out and active.proc.poll() is not None and (directory/'timeout-failure.json').is_file(),
                                           child_exit_code=active.proc.returncode,preserved_result='timeout-failure.json')
        active = None
        rows = report['requests']
        values = [r['decision_seconds'] for r in rows]
        report['summary'] = dict(requests=len(rows),passed=sum(r['passed'] for r in rows),
            correct_rate=sum(r.get('correct',False) for r in rows)/len(rows),
            json_valid_rate=sum(r.get('json_valid',False) for r in rows)/len(rows),
            schema_valid_rate=sum(r.get('schema_valid',False) for r in rows)/len(rows),
            eos_rate=sum(r.get('eos_reached',False) for r in rows)/len(rows),
            latency_seconds=dict(p50=percentile(values,0.5),p95=percentile(values,0.95),maximum=max(values),minimum=min(values)),
            controller_roundtrip_p95_seconds=percentile([r['controller_roundtrip_seconds'] for r in rows],0.95),
            cuda_peak_allocated_bytes=max(r['cuda_peak_allocated_bytes'] for r in rows),
            cuda_peak_reserved_bytes=max(r['cuda_peak_reserved_bytes'] for r in rows),
            endpoint_rss_first10_median_bytes=percentile([r['process_rss_bytes'] for r in rows[:10]],0.5),
            endpoint_rss_last10_median_bytes=percentile([r['process_rss_bytes'] for r in rows[-10:]],0.5))
        report['failures'] = [r for phase in ('startup','warmup','requests') for r in report[phase] if not r['passed']]
        report['all_passed'] = not report['failures'] and all(c['passed'] for c in report['checks'].values())
        report['p1_required_counts_met'] = args.startup_runs>=3 and args.warmup>=5 and args.requests>=100
        report['status'] = 'passed' if report['all_passed'] else 'failed'
    except BaseException as exc:
        report['status'] = 'failed'
        report['all_passed'] = False
        report['failures'].append(dict(error=type(exc).__name__,detail=str(exc)))
        traceback.print_exc()
    finally:
        if active is not None:
            active.close()
            report['resources'][active.name] = resource_summary(active.samples)
        raw_file.close()
        report['finished_utc'] = datetime.now(timezone.utc).isoformat()
        save(directory/'report.json',report)
    print(json.dumps(dict(status=report['status'],summary=report.get('summary'),directory=str(directory)),ensure_ascii=False),flush=True)
    if not report.get('all_passed'):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
