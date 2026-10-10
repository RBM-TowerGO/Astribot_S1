"""Observation only: never includes a chassis or simulation node."""
import os
from pathlib import Path
import numpy as np
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from towergo_real.contracts import calibration


def start(context):
    offline = LaunchConfiguration('offline').perform(context).lower() == 'true'
    expected = '125' if offline else '25'
    if os.environ.get('ROS_DOMAIN_ID') != expected:
        raise RuntimeError('Wrong ROS domain for this entry point')
    if offline and os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Offline processing requires localhost discovery')
    path = LaunchConfiguration('calibration').perform(context)
    c = calibration(path)
    t = np.asarray(c['imu_from_lidar'])
    share = Path(get_package_share_directory('towergo_real'))
    common = {'use_sim_time': offline}
    lio = dict(common, **{
        'feature_extract_enable': False, 'point_filter_num': 3, 'max_iteration': 3,
        'filter_size_surf': .15, 'filter_size_map': .2, 'cube_side_length': 200.,
        'runtime_pos_log_enable': False,
        'common.lid_topic': '/towergo/lio/lidar', 'common.imu_topic': '/towergo/lio/imu',
        'common.time_sync_en': False, 'common.time_offset_lidar_to_imu': 0.,
        'preprocess.lidar_type': 1, 'preprocess.scan_line': 4, 'preprocess.blind': float(c['range_min']),
        'preprocess.timestamp_unit': 3, 'preprocess.scan_rate': 10,
        'mapping.acc_cov': .1, 'mapping.gyr_cov': .1, 'mapping.b_acc_cov': .0001, 'mapping.b_gyr_cov': .0001,
        'mapping.fov_degree': 360., 'mapping.det_range': float(c['range_max']),
        'mapping.extrinsic_est_en': False, 'mapping.extrinsic_T': t[:3,3].tolist(),
        'mapping.extrinsic_R': t[:3,:3].ravel().tolist(),
        'publish.path_en': False, 'publish.scan_publish_en': True, 'publish.dense_publish_en': True,
        'publish.scan_bodyframe_pub_en': True, 'publish.map_en': False,
        'publish.effect_map_en': False, 'pcd_save.pcd_save_en': False,
    })
    adapter = Node(package='towergo_livox_adapter', executable='livox_adapter',
                   parameters=[common, {'imu_acceleration_unit': c['imu_acceleration_unit']}], output='screen')
    fast = Node(package='fast_lio', executable='fastlio_mapping', name='towergo_fast_lio',
                parameters=[lio], output='screen', remappings=[
                    ('/Odometry','/towergo/lio/odometry_raw'), ('/cloud_registered_body','/towergo/lio/cloud_body'),
                    ('/cloud_registered','/towergo/lio/cloud_world'), ('/tf','/towergo/lio/tf_raw'),
                    ('/tf_static','/towergo/lio/tf_static_raw'), ('/path','/towergo/lio/path_raw'),
                    ('/Laser_map','/towergo/lio/map_raw')])
    frames = Node(package='towergo_real', executable='lio_adapter',
                  parameters=[common, {'calibration': path}], output='screen')
    slam = Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
                parameters=[str(share/'config/slam_real.yaml'), common,
                            {'min_laser_range':float(c['range_min']), 'max_laser_range':float(c['range_max'])}], output='screen')
    nodes = [adapter, fast, frames, slam]
    if LaunchConfiguration('viewer').perform(context).lower() == 'true':
        nodes.append(Node(package='towergo_real', executable='map_viewer', parameters=[common], output='screen'))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=n, on_exit=[EmitEvent(event=Shutdown(reason='Mapping process exited; new session required'))]))
                for n in (adapter, fast, frames, slam)]
    return handlers + nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('calibration'),
                              DeclareLaunchArgument('offline', default_value='false'),
                              DeclareLaunchArgument('viewer', default_value='true'), OpaqueFunction(function=start)])
