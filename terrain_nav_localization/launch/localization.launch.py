import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('terrain_nav_localization'), 'config', 'ekf.yaml')

    ekf_local = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        output='screen',
        parameters=[config],
        remappings=[('odometry/filtered', 'odometry/local')],
    )

    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global',
        output='screen',
        parameters=[config],
        remappings=[('odometry/filtered', 'odometry/global')],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[config],
        remappings=[
            ('imu', 'imu/data'),
            ('gps/fix', 'gps/fix'),
            ('odometry/filtered', 'odometry/global'),
        ],
    )

    return LaunchDescription([ekf_local, ekf_global, navsat])
