"""The sole SDK-import boundary. Not imported by observation or mapping nodes."""
import fcntl
import builtins
import importlib.util
import os
from pathlib import Path
import sys
from .contracts import hardware_contract


def construct_without_interactive_takeover(sdk_class, **kwargs):
    """Decline the vendor takeover prompt, including in non-interactive shells.

    This SDK catches EOFError as a reason to acquire control. An empty answer,
    not EOF or an exception, is therefore required. This process has no keyboard
    input; the keyboard node runs in a different process.
    """
    original_input = builtins.input
    try:
        builtins.input = lambda *args, **kwargs: ''
        return sdk_class(**kwargs)
    finally:
        builtins.input = original_input


class AstribotPositionBackend:
    def __init__(self, contract_path, feedback, rights_conflict):
        # Validate everything before importing or constructing the SDK.
        self.config, self.limits = hardware_contract(contract_path)
        if os.environ.get('ROS_DOMAIN_ID') != '25' or os.environ.get('ROBOT_TYPE') != 'S1':
            raise RuntimeError('Hardware requires Domain 25 and sourced S1 SDK environment')
        if rights_conflict():
            raise RuntimeError('Existing SDK control-rights owner; use the validated handover procedure')
        self.feedback = feedback
        self.sdk = None
        lockdir = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'))
        self.lock = (lockdir / f'towergo-chassis-{os.getuid()}.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            spec = importlib.util.spec_from_file_location('towergo_validated_stop', self.config['_stop_file'])
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self.stop_impl = module.stop_chassis
            sys.path.insert(0, self.config['sdk_root'])
            # A second owner appearing between discovery and construction must
            # never receive an automatic "yes" answer from this process.
            from astribot_sdk.core.astribot_api.astribot_client import Astribot
            self.sdk = construct_without_interactive_takeover(
                Astribot, freq=250., high_control_rights=False, node_name='towergo_chassis_sdk')
            if not self.sdk.get_control_rights_status():
                raise RuntimeError('SDK did not acquire control rights')
        except Exception:
            self.close()
            raise

    def position(self):
        return self.feedback()

    def has_control(self):
        return bool(self.sdk and self.sdk.get_control_rights_status())

    def send(self, position):
        if not self.has_control():
            raise RuntimeError('SDK control rights lost')
        self.sdk.set_joints_position([self.sdk.chassis_name], [position],
                                    control_way='filter', use_wbc=False, add_default_torso=False)

    def stop(self):
        if not self.has_control():
            raise RuntimeError('Cannot issue chassis stop after losing SDK control rights')
        # Implemented only after the vendor procedure is verified, including
        # stale feedback. A no-op/last-target hold is not supplied as a default.
        self.stop_impl(self.sdk)

    def close(self):
        try:
            if self.sdk is not None:
                self.sdk.astribot_interface.shutdown()
                self.sdk = None
        finally:
            if getattr(self, 'lock', None):
                self.lock.close()
                self.lock = None
