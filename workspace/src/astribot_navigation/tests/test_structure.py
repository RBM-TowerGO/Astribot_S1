from pathlib import Path
import ast,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]

def test_python_parses_and_packages_exist():
    for path in ROOT.rglob('*.py'):ast.parse(path.read_text())
    assert {p.parent.name for p in ROOT.glob('*/package.xml')} == {
        'astribot_nav_api', 'astribot_nav_bridge', 'astribot_nav_bringup',
        'astribot_nav_sensors', 'astribot_nav_sim', 'towergo_real', 'towergo_livox_adapter'}
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
