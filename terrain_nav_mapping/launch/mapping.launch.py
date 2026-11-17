import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('terrain_nav_mapping'), 'config', 'elevation_mapper.yaml')

    return LaunchDescription([
        Node(
            package='terrain_nav_mapping',
            executable='elevation_mapper_node',
            name='elevation_mapper',
            output='screen',
            parameters=[config],
            remappings=[('points', 'lidar/points')],
        ),
    ])
