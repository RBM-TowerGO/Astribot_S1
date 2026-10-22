from pathlib import Path
import ast,xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[1]

def test_python_parses_and_packages_exist():
    for path in ROOT.rglob('*.py'):ast.parse(path.read_text())
    assert {p.parent.name for p in ROOT.glob('*/package.xml')} == {
        'astribot_nav_api', 'astribot_nav_bridge', 'astribot_nav_bringup',
        'astribot_nav_sensors', 'astribot_nav_sim', 'towergo_real', 'towergo_livox_adapter', 'towergo_dual_livox'}
    for path in ROOT.glob('*/package.xml'):ET.parse(path)
def test_no_hardware_sdk_imports():
    for folder in ROOT.glob('astribot_nav_*'):
        for path in folder.rglob('*.py'):
            tree=ast.parse(path.read_text())
            for n in ast.walk(tree):
                if isinstance(n,ast.Import):assert all(not x.name.startswith('astribot_sdk') for x in n.names)
                if isinstance(n,ast.ImportFrom):assert not (n.module or '').startswith('astribot_sdk')
def test_no_host_network_or_device_mount():
    text=(ROOT/'scripts/container.sh').read_text()
    assert '--network none' in text
    assert '--network host' not in text and '--device ' not in text

def test_sdk_import_is_confined_to_explicit_backend():
    root = ROOT/'towergo_real'/'towergo_real'
    for path in root.glob('*.py'):
        if path.name == 'sdk_backend.py':
            continue
        for n in ast.walk(ast.parse(path.read_text())):
            if isinstance(n, ast.Import):
                assert all(not x.name.startswith('astribot_sdk') for x in n.names)
            if isinstance(n, ast.ImportFrom):
                assert not (n.module or '').startswith('astribot_sdk')


def test_offline_navigation_speed_is_capped_at_point_one():
    config = yaml.safe_load((ROOT/'astribot_nav_bringup/config/nav2.yaml').read_text())
    follow = config['controller_server']['ros__parameters']['FollowPath']
    assert follow['max_vel_x'] == .1
    assert follow['max_speed_xy'] == .1
    assert follow['max_vel_theta'] == .1
    assert follow['rotate_to_heading_angular_vel'] == .1
    behavior = config['behavior_server']['ros__parameters']
    assert behavior['max_rotational_vel'] == .1


def test_web_path_follower_keeps_low_speed_and_collision_checks():
    config=yaml.safe_load((ROOT/'astribot_nav_bringup/config/web_controller.yaml').read_text())
    follow=config['controller_server']['ros__parameters']['FollowPath']
    assert 0 < follow['regulated_linear_scaling_min_speed'] <= follow['desired_linear_vel'] <= .1
    assert 0 < follow['rotate_to_heading_angular_vel'] <= .1
    assert follow['use_collision_detection'] and follow['use_rotate_to_heading']
    assert not follow['allow_reversing']
