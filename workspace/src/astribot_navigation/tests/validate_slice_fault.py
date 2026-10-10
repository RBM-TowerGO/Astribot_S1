#!/usr/bin/env python3
"""Explicit Domain-125 replay fault injection into our own slice child only."""
import argparse,json,os
from pathlib import Path
import signal,subprocess,tempfile,time


def main():
    p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('output_root');args=p.parse_args()
    root=Path(tempfile.mkdtemp(prefix='slice_fault_',dir=args.output_root))
    script=Path(__file__).resolve().parents[1]/'scripts/tg-mapping.sh'
    with (root/'test.log').open('w') as log:
        proc=subprocess.Popen([str(script),'--replay',args.bag,'--port','18769','--output-root',str(root)],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            deadline=time.monotonic()+90;run=None
            while time.monotonic()<deadline:
                assert proc.poll() is None,'Replay exited before injected fault'
                for d in root.glob('mapping_*'):
                    try:s=json.loads((d/'session.json').read_text())
                    except (OSError,ValueError):continue
                    if s['status']=='mapping':run=d;break
                if run:break
                time.sleep(.1)
            assert run,'Never reached READY'
            # PID comes from this newly launched manager, never process-name selection.
            pid=s['processes']['slice']['pid']
            command=(Path('/proc')/str(pid)/'cmdline').read_bytes()
            assert b'mapping_slice_node.py' in command and str(run).encode() in command
            os.kill(pid,signal.SIGTERM)
            assert proc.wait(timeout=45)==2
            result=json.loads((run/'session.json').read_text())
            assert result['status']=='incomplete' and 'slice' in result['error'].lower()
            assert result['hardware_commands_sent'] is False and result['replay']
            assert not result['shutdown_errors']
            assert result['raw_counts'] and result['lio_counts']
            for child in result['processes'].values():
                assert not (Path('/proc')/str(child['pid'])).exists()
            report=dict(passed=True,session=str(run),error=result['error'],shutdown_errors=result['shutdown_errors'])
            (root/'validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=45)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()


if __name__=='__main__':main()
