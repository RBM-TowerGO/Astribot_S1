import argparse
import json
from .contracts import hardware_contract, calibration


def main():
    parser = argparse.ArgumentParser(description='Validate evidence without importing the SDK or moving hardware')
    parser.add_argument('--hardware-contract')
    parser.add_argument('--calibration')
    args = parser.parse_args()
    if not args.hardware_contract and not args.calibration:
        parser.error('Provide --hardware-contract and/or --calibration')
    report = {}
    for key, path, check in [('hardware', args.hardware_contract, hardware_contract),
                             ('calibration', args.calibration, calibration)]:
        if path:
            try:
                check(path)
                report[key] = {'passed': True}
            except Exception as exc:
                report[key] = {'passed': False, 'reason': str(exc)}
    print(json.dumps(report, indent=2, ensure_ascii=False))
    raise SystemExit(0 if all(r['passed'] for r in report.values()) else 2)


if __name__ == '__main__':
    main()
