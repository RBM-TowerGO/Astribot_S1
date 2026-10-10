"""Offline two-image smoke test using the shared P1 inference implementation."""
import argparse
import json
from datetime import datetime, timezone
from inference_core import ROOT, InferenceEngine, read_config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dtype', choices=['float16','bfloat16'])
    parser.add_argument('--output', default=None)
    args = parser.parse_args()
    cfg = read_config()
    if args.dtype:
        cfg['dtype'] = args.dtype
    output = Path(args.output) if args.output else ROOT/'reports'/('smoke-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))/'smoke-inference.json'
    if not output.is_absolute():
        output = ROOT/output
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise FileExistsError(output)
    engine = InferenceEngine(cfg)
    results = []
    for expected in ('A','B'):
        result = engine.infer(f'assets/samples/red_{expected}.png', expected)
        results.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    report = dict(test='synthetic image smoke, not navigation accuracy',
                  timestamp_utc=datetime.now(timezone.utc).isoformat(), config=cfg,
                  load_seconds=engine.load_seconds, results=results,
                  all_passed=all(r['passed'] for r in results))
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    if not report['all_passed']:
        raise SystemExit('Smoke test failed; see report')


if __name__ == '__main__':
    from pathlib import Path
    main()
