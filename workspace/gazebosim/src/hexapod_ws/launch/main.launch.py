#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_name  = 'tiffany_sim'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_path    = os.path.join(pkg_share, 'description', 'hexapod.urdf.xacro')
    world_path   = os.path.join(pkg_share, 'worlds', 'hexapod.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    params_file  = os.path.join(pkg_share, 'config', 'parameters.yaml')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': [f'-r -v4 {world_path}'],
            'on_exit_shutdown': 'true'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'tiffany', '-z', '0.2'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen'
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    tiffany_ctrl_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tiffany_controller', '--param-file', params_file],
        output='screen'
    )

    tiffany_brain = Node(
        package=pkg_name,
        executable='hexapod_runner.py',
        name='tiffany_runner',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_robot,
        bridge,
        TimerAction(period=5.0,  actions=[jsb_spawner]),
        TimerAction(period=10.0, actions=[tiffany_ctrl_spawner]),
        TimerAction(period=15.0, actions=[tiffany_brain]),
    ])