"""Recorded-map browser simulation; never launches hardware or vendor SDK."""
import os
import tempfile
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction, SetLaunchConfiguration, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnShutdown
from astribot_nav_sim.grid_world import GridWorld


def validate(context):
    if os.environ.get('ROS_DOMAIN_ID') != '125' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Web navigation requires isolated simulation Domain 125 / localhost')
    if not Path(LaunchConfiguration('map').perform(context)).is_file():
        raise RuntimeError('Recorded map YAML does not exist')
    return []


def prepare_collision_map(context):
    world=GridWorld(LaunchConfiguration('map').perform(context))
    if abs(world.resolution-.05)>1e-8:
        raise RuntimeError('Web differential lattice profile requires a 0.05 m map resolution')
    temporary=tempfile.TemporaryDirectory(prefix='towergo-collision-map-')
    path=Path(temporary.name)
    world.collision_image().save(path/'map.pgm')
    (path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=world.resolution,
        origin=world.origin.tolist(),negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary')))
    def cleanup(context):
        temporary.cleanup()
        return []
    return [SetLaunchConfiguration('collision_map',str(path/'map.yaml')),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]


def generate_launch_description():
    share = get_package_share_directory('astribot_nav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map'), DeclareLaunchArgument('port', default_value='8766'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        OpaqueFunction(function=validate),
        OpaqueFunction(function=prepare_collision_map),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share)/'launch/offline_navigation.launch.py')),
            launch_arguments={'max_linear_speed':'0.3', 'map':LaunchConfiguration('map'), 'sim_map':LaunchConfiguration('map'),
                              'collision_map':LaunchConfiguration('collision_map'),
                              'navigation_bt':str(Path(share)/'config/web_navigation.xml'),
                              'rate':LaunchConfiguration('rate'), 'noise':'true', 'scan_source':'dual',
                              'footprint_padding':'0.06', 'inflation_radius':'0.60',
                              'cost_scaling_factor':'5.0',
                              'controller_config':str(Path(share)/'config/web_controller.yaml')}.items()),
        ExecuteProcess(cmd=['/usr/bin/python3', str(Path(share)/'web/nav_web.py'),
                            '--map', LaunchConfiguration('map'), '--port', LaunchConfiguration('port')], output='screen'),
    ])
