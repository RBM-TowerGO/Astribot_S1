"""Fail-closed, evidence-backed hardware and calibration configuration."""
from pathlib import Path
import hashlib
import math
import os
import subprocess
import yaml
from .control import Limits


def load_yaml(path):
    with Path(path).open() as f:
        value = yaml.safe_load(f)
    if not isinstance(value, dict):
        raise ValueError('Configuration must be a mapping')
    return value


def evidence(config, fields, base):
    if config.get('verified') is not True:
        raise ValueError('Not field-verified; hardware/calibrated mapping remains disabled')
    if not config.get('reviewed_by') or not config.get('reviewed_at'):
        raise ValueError('Reviewer and date are required')
    for field in fields:
        row = config.get('evidence', {}).get(field, {})
        path = Path(row.get('path') or '')
        if not path.is_absolute():
            path = base / path
        if not path.is_file() or not row.get('sha256'):
            raise ValueError(f'Missing evidence file/hash: {field}')
        if hashlib.sha256(path.read_bytes()).hexdigest() != row['sha256']:
            raise ValueError(f'Evidence hash mismatch: {field}')


HARDWARE_EVIDENCE = ('coordinate_semantics', 'feedback_semantics', 'stop_procedure',
                     'process_crash_watchdog', 'control_handover', 'cycle_and_tracking_limits')
CALIBRATION_EVIDENCE = ('base_lidar_extrinsic', 'lidar_imu_extrinsic', 'imu_units',
                        'body_filter', 'clock_alignment')


def hardware_contract(path):
    config = load_yaml(path)
    evidence(config, HARDWARE_EVIDENCE, Path(path).resolve().parent)
    if config.get('position_semantics') != 'sdk202_local_integrated_m_m_rad':
        raise ValueError('Only the field-validated SDK 202 coordinate convention is implemented')
    if config.get('feedback_topic') != '/astribot_chassis/joint_space_states':
        raise ValueError('Unsupported feedback source; implement and validate its adapter first')
    # No default watchdog/following-error values are allowed for hardware.
    required = ('max_cycle', 'feedback_timeout', 'max_position_error', 'max_yaw_error')
    values = config.get('limits', {})
    if any(values.get(k) is None for k in required):
        raise ValueError('Hardware timing/tracking limits require field evidence')
    limits = Limits(**values)
    sdk_root = Path(config['sdk_root']).resolve()
    commit = subprocess.check_output(['git', '-C', str(sdk_root), 'rev-parse', 'HEAD'], text=True).strip()
    if commit != config.get('sdk_commit'):
        raise ValueError('SDK commit differs from the verified version')
    if subprocess.check_output(['git', '-C', str(sdk_root), 'diff', '--name-only'], text=True).strip():
        raise ValueError('SDK has tracked modifications; revalidate before enabling')
    stop = config.get('stop_plugin', {})
    file = Path(stop.get('file') or '')
    if not file.is_absolute():
        file = Path(path).resolve().parent / file
    if not file.is_file() or hashlib.sha256(file.read_bytes()).hexdigest() != stop.get('sha256'):
        raise ValueError('A reviewed, checksum-pinned chassis stop implementation is required')
    config['_stop_file'] = str(file)
    return config, limits


def calibration(path):
    import numpy as np
    config = load_yaml(path)
    if config.get('test_only') and os.environ.get('ROS_DOMAIN_ID') != '125':
        raise ValueError('Synthetic calibration is restricted to Domain 125')
    evidence(config, CALIBRATION_EVIDENCE, Path(path).resolve().parent)
    for key in ('base_from_lidar', 'imu_from_lidar'):
        t = np.asarray(config.get(key), dtype=float)
        if t.shape != (4, 4) or not np.isfinite(t).all() or not np.allclose(t[3], [0, 0, 0, 1]):
            raise ValueError(f'{key} must be a finite 4x4 rigid transform')
        if not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(t[:3, :3]), 1):
            raise ValueError(f'{key} rotation is not SO(3)')
    polygon = np.asarray(config.get('body_polygon'), dtype=float)
    if polygon.ndim != 2 or polygon.shape[1] != 2 or len(polygon) < 3 or not np.isfinite(polygon).all():
        raise ValueError('Measured body polygon is required')
    for key in ('obstacle_height_min', 'obstacle_height_max', 'range_min', 'range_max',
                'max_pose_step_m', 'max_pose_step_rad'):
        if not isinstance(config.get(key), (float, int)) or not math.isfinite(config[key]):
            raise ValueError(f'Measured/validated parameter required: {key}')
    if not 0 <= config['obstacle_height_min'] < config['obstacle_height_max']:
        raise ValueError('Invalid obstacle height interval')
    if not 0 < config['range_min'] < config['range_max'] or min(config['max_pose_step_m'], config['max_pose_step_rad']) <= 0:
        raise ValueError('Invalid range or discontinuity limits')
    if config.get('imu_acceleration_unit') not in ('g', 'm_s2') or config.get('imu_gyro_unit') != 'rad_s':
        raise ValueError('Explicit verified IMU units are required')
    if config.get('point_timestamp_unit') != 'absolute_ns':
        raise ValueError('Only measured absolute-nanosecond Livox input is supported')
    return config
