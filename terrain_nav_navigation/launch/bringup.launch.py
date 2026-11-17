import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(pkg, launch_file, args=None):
    path = os.path.join(get_package_share_directory(pkg), 'launch', launch_file)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(args or {}).items())


def generate_launch_description():
    nav_pkg = get_package_share_directory('terrain_nav_navigation')
    lc = LaunchConfiguration

    args = [
        DeclareLaunchArgument('mode', default_value='terrain', choices=['terrain', 'default']),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x', default_value='-30.0'),
        DeclareLaunchArgument('y', default_value='20.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    sim = _include('terrain_nav_gazebo', 'sim.launch.py', {
        'headless': lc('headless'), 'rviz': 'false', 'publish_odom_tf': 'false',
        'x': lc('x'), 'y': lc('y'), 'yaw': lc('yaw'),
    })
    localization = _include('terrain_nav_localization', 'localization.launch.py')
    mapping = _include('terrain_nav_mapping', 'mapping.launch.py')
    navigation = _include('terrain_nav_navigation', 'navigation.launch.py', {'mode': lc('mode')})

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(nav_pkg, 'rviz', 'nav.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(lc('rviz')),
    )

    return LaunchDescription(args + [
        sim,
        TimerAction(period=6.0, actions=[localization, mapping]),
        TimerAction(period=9.0, actions=[navigation, rviz]),
    ])
