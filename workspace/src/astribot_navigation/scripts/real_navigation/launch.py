"""Launch only navigation nodes in a private namespace and TF tree."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    filename=LaunchConfiguration('params').perform(context)
    managed=[];result=[]
    for package,executable,name in [('nav2_map_server','map_server','map_server'),('nav2_amcl','amcl','amcl'),
                                   ('nav2_planner','planner_server','planner_server'),('nav2_controller','controller_server','controller_server'),
                                   ('nav2_behaviors','behavior_server','behavior_server'),('nav2_bt_navigator','bt_navigator','bt_navigator')]:
        result.append(Node(package=package,executable=executable,name=name,namespace='/towergo/nav',
                           parameters=[filename],remappings=[('/tf','/towergo/nav/tf'),('/tf_static','/towergo/nav/tf_static'),('cmd_vel','cmd_vel_nav')],output='screen'))
        managed.append(name)
    result.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',namespace='/towergo/nav',
                       parameters=[{'use_sim_time':False,'autostart':True,'node_names':managed,'bond_timeout':4.0}]))
    return result


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('params'),OpaqueFunction(function=nodes)])
