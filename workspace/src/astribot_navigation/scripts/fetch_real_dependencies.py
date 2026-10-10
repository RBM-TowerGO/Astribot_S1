#!/usr/bin/env python3
"""Pinned source checkout and interface-only Livox overlay; no system edits."""
import argparse
import json
from pathlib import Path
import subprocess

FAST_LIO_COMMIT = '2fffc570a25d0df172720bac034fbdb6a13d2162'


def run(*args):
    return subprocess.check_output(args, text=True).strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', required=True)
    args = parser.parse_args()
    root = Path(args.directory).resolve(); root.mkdir(parents=True, exist_ok=True)
    source = root/'fast_lio'
    if not source.exists():
        subprocess.run(['git', 'clone', 'https://github.com/Ericsii/FAST_LIO_ROS2.git', str(source)], check=True)
    if run('git', '-C', str(source), 'status', '--porcelain'):
        raise SystemExit('Dependency checkout contains local edits; refusing to overwrite')
    subprocess.run(['git', '-C', str(source), 'checkout', '--detach', FAST_LIO_COMMIT], check=True)
    subprocess.run(['git', '-C', str(source), 'submodule', 'update', '--init', '--recursive'], check=True)
    # Installed vendor package exports a non-library executable as a library.
    # Build identical ROS interfaces in a separate overlay rather than touching it.
    iface = root/'livox_ros_driver2'; (iface/'msg').mkdir(parents=True, exist_ok=True)
    files = {
        'msg/CustomMsg.msg': 'std_msgs/Header header\nuint64 timebase\nuint32 point_num\nuint8 lidar_id\nuint8[3] rsvd\nCustomPoint[] points\n',
        'msg/CustomPoint.msg': 'uint32 offset_time\nfloat32 x\nfloat32 y\nfloat32 z\nuint8 reflectivity\nuint8 tag\nuint8 line\n',
        'package.xml': '''<?xml version="1.0"?>
<package format="3"><name>livox_ros_driver2</name><version>1.0.0</version>
<description>Interface-only overlay matching Livox ROS Driver 2; no hardware driver.</description>
<maintainer email="maintainer@example.invalid">TowerGO</maintainer><license>BSD-3-Clause</license>
<buildtool_depend>ament_cmake</buildtool_depend><buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>std_msgs</depend><exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group><export><build_type>ament_cmake</build_type></export></package>
''',
        'CMakeLists.txt': '''cmake_minimum_required(VERSION 3.8)
project(livox_ros_driver2)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/CustomMsg.msg" "msg/CustomPoint.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
'''}
    for name, content in files.items():
        p = iface/name
        if p.exists() and p.read_text() != content:
            raise SystemExit(f'Refusing to overwrite modified dependency interface: {p}')
        p.write_text(content)
    manifest = {'fast_lio': {'url': 'https://github.com/Ericsii/FAST_LIO_ROS2.git', 'commit': FAST_LIO_COMMIT,
                              'submodules': run('git', '-C', str(source), 'submodule', 'status', '--recursive')},
                'livox_interfaces': 'CustomMsg/CustomPoint wire definitions only; system driver unchanged'}
    (root/'source_versions.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
