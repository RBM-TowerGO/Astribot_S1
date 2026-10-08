"""Collect coordinate diagnostics for reporting; never gate startup on approval."""
import hashlib
import json
from pathlib import Path

CONTRACT='both_raw_topics_already_in_front_lidar_frame'
CONFIG=Path('/opt/astribot_ros/software/livox_ros_driver2/share/config/MID360_config.json')
EVIDENCE=Path(__file__).resolve().parents[1]/'towergo_dual_livox/config/coordinate_verification.json'

def coordinate_diagnostics(config=CONFIG, evidence=EVIDENCE):
    notes=[]
    try:
        report=json.loads(Path(evidence).read_text())
        if not isinstance(report,dict): raise ValueError('Expected a JSON object')
    except (OSError,ValueError) as exc:
        report={}
        notes.append('标定记录不可用：'+str(exc))
    if report.get('coordinate_contract')!=CONTRACT or report.get('frame_convention_verified') is not True:
        notes.append('坐标约定尚无确认记录；沿用驱动已转换到前雷达坐标系的输入约定。')
    try:
        current=hashlib.sha256(Path(config).read_bytes()).hexdigest()
        report['current_driver_config_sha256']=current
        if current!=report.get('driver_config_sha256'):
            notes.append('驱动配置与历史标定记录不同。')
    except OSError as exc:
        notes.append('驱动配置快照不可用：'+str(exc))
    report['live_alignment_verified']=report.get('live_alignment_verified') is True
    if not report['live_alignment_verified']:
        notes.append('标定精度尚未验收；此信息仅供参考，不限制建图或遥操。')
    report['startup_restricted_by_calibration']=False
    report['diagnostic_notes']=notes
    return report
