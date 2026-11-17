import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    pkg = get_package_share_directory('terrain_nav_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = PathJoinSubstitution([
        pkg, 'config',
        [TextSubstitution(text='nav2_'), LaunchConfiguration('mode'), TextSubstitution(text='.yaml')],
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='terrain', choices=['terrain', 'default'],
            description='terrain: traversability costmap layer; default: 2D obstacle layer'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': params_file,
                'autostart': 'true',
            }.items(),
        ),
    ])
