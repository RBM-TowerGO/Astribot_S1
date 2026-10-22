import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def check(context):
    if not Path(LaunchConfiguration('map').perform(context)).is_file(): raise RuntimeError('map must be an existing absolute YAML path')
    return []

def navigation_nodes(context):
    p=get_package_share_directory('astribot_nav_bringup')
    config=RewrittenYaml(source_file=os.path.join(p,'config','nav2.yaml'), root_key='',
        param_rewrites={key:LaunchConfiguration(key) for key in ('footprint_padding','inflation_radius','cost_scaling_factor')},convert_types=True)
    nodes=[]
    controller_config=LaunchConfiguration('controller_config').perform(context)
    if controller_config and not Path(controller_config).is_file():
        raise RuntimeError('controller_config must be an existing YAML file')
    managed=[]
    collision_map=LaunchConfiguration('collision_map').perform(context)
    if collision_map:
        if not Path(collision_map).is_file(): raise RuntimeError('Collision map does not exist')
        nodes.append(Node(package='nav2_map_server',executable='map_server',name='collision_map_server',
            parameters=[{'use_sim_time':True,'yaml_filename':collision_map,'frame_id':'map'}],
            remappings=[('map','/sim/collision_map')],output='screen'))
        managed.append('collision_map_server')
    for package,exe,name in [('nav2_map_server','map_server','map_server'),('nav2_amcl','amcl','amcl'),('nav2_planner','planner_server','planner_server'),('nav2_controller','controller_server','controller_server'),('nav2_behaviors','behavior_server','behavior_server'),('nav2_bt_navigator','bt_navigator','bt_navigator')]:
        extra={}
        if name=='map_server': extra['yaml_filename']=LaunchConfiguration('map')
        if name=='bt_navigator':
            extra['default_nav_to_pose_bt_xml']=LaunchConfiguration('navigation_bt').perform(context) or os.path.join(p,'config','navigation.xml')
            extra['default_nav_through_poses_bt_xml']=os.path.join(p,'config','navigation_through.xml')
            extra['plugin_lib_names']=['nav2_compute_path_through_poses_action_bt_node','nav2_compute_path_to_pose_action_bt_node','nav2_follow_path_action_bt_node','nav2_pipeline_sequence_bt_node','nav2_rate_controller_bt_node','nav2_recovery_node_bt_node','nav2_wait_action_bt_node']
        parameters=[config,extra]
        if name in ('controller_server','planner_server') and controller_config: parameters.append(controller_config)
        nodes.append(Node(package=package,executable=exe,name=name,parameters=parameters,remappings=[('cmd_vel','/cmd_vel_nav')],output='screen'));managed.append(name)
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',parameters=[{'use_sim_time':True,'autostart':True,'node_names':managed,'bond_timeout':4.0}],output='screen'))
    return nodes


def generate_launch_description():
    p=get_package_share_directory('astribot_nav_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('map'),DeclareLaunchArgument('sim_map',default_value=''),
        DeclareLaunchArgument('controller_config',default_value=''),
        DeclareLaunchArgument('collision_map',default_value=''),
        DeclareLaunchArgument('navigation_bt',default_value=''),
        DeclareLaunchArgument('footprint_padding',default_value='0.06'),
        DeclareLaunchArgument('inflation_radius',default_value='0.65'),
        DeclareLaunchArgument('cost_scaling_factor',default_value='3.0'),
        OpaqueFunction(function=check),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(p,'launch','simulation.launch.py')),
            launch_arguments={'mapping':'false','map_yaml':LaunchConfiguration('sim_map')}.items()),
        OpaqueFunction(function=navigation_nodes)])
