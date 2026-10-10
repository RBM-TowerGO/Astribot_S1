"""Private line-JSON worker protocol; stdout reserved for benchmark events."""
import contextlib
import json
import sys
import traceback


def emit(event):
    print(json.dumps(event, ensure_ascii=False), flush=True)


def main():
    try:
        with contextlib.redirect_stdout(sys.stderr):
            from inference_core import InferenceEngine
            engine = InferenceEngine()
        emit(dict(event='ready', load_seconds=engine.load_seconds, **engine.load_memory))
        for line in sys.stdin:
            command = json.loads(line)
            with contextlib.redirect_stdout(sys.stderr):
                result = engine.infer(**command)
            emit(dict(event='result', result=result))
    except Exception as exc:
        traceback.print_exc(file=sys.stderr)
        emit(dict(event='fatal', error=type(exc).__name__, detail=str(exc)))
        raise SystemExit(1)


if __name__ == '__main__':
    main()
