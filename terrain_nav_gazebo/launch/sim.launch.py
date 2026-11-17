import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('terrain_nav_gazebo')
    desc_pkg = get_package_share_directory('terrain_nav_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    x, y, z, yaw = (LaunchConfiguration(n) for n in ('x', 'y', 'z', 'yaw'))

    declare_args = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(gazebo_pkg, 'worlds', 'empty_terrain.sdf'),
            description='Absolute path to the SDF world file'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server only (no GUI)'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.3'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(desc_pkg, 'urdf', 'rover.urdf.xacro')]),
        value_type=str)

    gz_args = PythonExpression([
        "'-r -s -v 2 ' if '", headless, "' == 'true' else '-r -v 2 '"
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [gz_args, world]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'terrain_rover',
                   '-topic', 'robot_description',
                   '-x', x, '-y', y, '-z', z, '-Y', yaw],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(gazebo_pkg, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution(
            [desc_pkg, 'rviz', 'rover.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(declare_args + [
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz_node,
    ])
