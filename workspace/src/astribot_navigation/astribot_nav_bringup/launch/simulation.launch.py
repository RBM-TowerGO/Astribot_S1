import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def check(context):
    if os.environ.get('ROS_DOMAIN_ID')!='125': raise RuntimeError('Use the offline container with ROS_DOMAIN_ID=125')
    return []

def generate_launch_description():
    args=[DeclareLaunchArgument(k,default_value=v) for k,v in [('max_linear_speed','0.1'),('scenario','room'),('seed','42'),('noise','false'),('rate','2.0'),('mapping','false'),('scan_source','front'),('map_yaml','')]]
    params={'use_sim_time':True,**{k:ParameterValue(LaunchConfiguration(k),value_type=t) for k,t in [('max_linear_speed',float),('scenario',str),('seed',int),('noise',bool),('rate',float),('mapping',bool),('map_yaml',str)]}}
    return LaunchDescription(args+[OpaqueFunction(function=check),
      Node(package='astribot_nav_sim',executable='simulator',parameters=[params],output='screen'),
      Node(package='astribot_nav_sensors',executable='scan_processor',parameters=[{'use_sim_time':True,'scan_source':LaunchConfiguration('scan_source')}],output='screen'),
      Node(package='astribot_nav_bridge',executable='command_guard',parameters=[{'use_sim_time':True,'backend':'sim','max_linear_speed':ParameterValue(LaunchConfiguration('max_linear_speed'),value_type=float)}],output='screen')])
