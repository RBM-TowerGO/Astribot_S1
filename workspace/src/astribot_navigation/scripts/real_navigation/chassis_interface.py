"""SDK command boundary; importing this module never loads or constructs SDK.

Uses the keyboard teleop's position contract. Stop is the vendor whole-robot
stop request, not a measured brake guarantee. Never automatically restarts.
"""
import math


def position(value):
    p = [float(x) for x in value]
    if len(p) != 3 or not all(math.isfinite(x) for x in p):
        raise ValueError('底盘目标必须是有限的 [x, y, yaw]')
    return p


class ChassisInterface:
    def __init__(self, sdk, stop_impl):
        self.sdk, self.stop_impl = sdk, stop_impl
        self.stopped = False
        self.commanded = False

    def has_control(self):
        return bool(self.sdk.get_control_rights_status())

    def desired(self):
        return position(self.sdk.get_desired_joints_position([self.sdk.chassis_name])[0])

    def feedback(self):
        return position(self.sdk.get_current_joints_position([self.sdk.chassis_name])[0])

    def send(self, target):
        if self.stopped:
            raise RuntimeError('停车已锁定执行器；需重新启动会话')
        if not self.has_control():
            raise RuntimeError('底盘控制权丢失')
        target = position(target)
        self.commanded = True
        self.sdk.set_joints_position([self.sdk.chassis_name], [target],
                                    control_way='filter', use_wbc=False, add_default_torso=False)

    def stop(self):
        if self.stopped:
            return
        # An idle stop must not poison a session that has never sent a target.
        if not self.commanded:
            return
        # Latch before calling potentially blocking/failing vendor code.
        self.stopped = True
        if not self.has_control():
            raise RuntimeError('控制权丢失；未发送停车请求')
        self.stop_impl(self.sdk)
