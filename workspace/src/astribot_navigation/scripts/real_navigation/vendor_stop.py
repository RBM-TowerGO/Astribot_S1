"""Vendor whole-robot stop request. Physical stopping remains UNVERIFIED.

This may stop other joints as well as the chassis. No restart_robot, Home,
activation, feedback-target substitution, or last-target hold is performed.
The caller must latch the session and require an explicit restart after use.
"""

def stop_chassis(sdk):
    if not sdk.get_control_rights_status():
        raise RuntimeError('没有控制权；不发送整机停止请求')
    sdk.stop_robot()
