# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    map_path_3d=os.path.join(
        os.path.expanduser('~'), 'maps', 'iscas_museum', 'iscas_museum_3d.pcd'
    )
    map_path_2d=os.path.join(
        os.path.expanduser('~'), 'maps', 'iscas_museum', 'iscas_museum_2d.yaml'
    )
    param_file_name='mugimaru.param.yaml'

    rviz_config_path=PathJoinSubstitution([
        FindPackageShare('mugimaru_launcher'), 'config', 'mugimaru.rviz'
    ])

    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
    )

    define_world=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='define_world',
        namespace='',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'world', '--child-frame-id', 'map',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )
    define_livox_frame=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='define_livox_frame',
        namespace='',
        arguments=[
            '--x', '0.1012', '--y', '0', '--z', '0.177',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_link', '--child-frame-id', 'livox_frame',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz2=Node(
        package='rviz2',
        executable='rviz2',
        name='mugimaru_rviz2',
        namespace='',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        prefix='xterm -e',
    )

    twist_launch_path=PathJoinSubstitution([
        FindPackageShare('ros2_odometry_twist_converter'),
        'launch', 'mugimaru.launch.py'
    ])
    gyro_launch_path=PathJoinSubstitution([
        FindPackageShare('gyro_odometer'),
        'launch', 'mugimaru.launch.py'
    ])
    map_launch_path=PathJoinSubstitution([
        FindPackageShare('map_loader'),
        'launch', 'mugimaru.launch.py'
    ])
    points_launch_path=PathJoinSubstitution([
        FindPackageShare('pointcloud_preprocessor'),
        'launch', 'mugimaru.launch.py'
    ])
    ndt_launch_path=PathJoinSubstitution([
        FindPackageShare('ndt_scan_matcher'),
        'launch', 'mugimaru.launch.py'
    ])
    ekf_launch_path=PathJoinSubstitution([
        FindPackageShare('ekf_localizer'),
        'launch', 'mugimaru.launch.py'
    ])
    nav2_launch_path=PathJoinSubstitution([
        FindPackageShare('mugimaru_navigation2'),
        'launch', 'mugimaru.launch.py'
    ])

    ld_twist=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(twist_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    ld_gyro=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gyro_launch_path),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    ld_map=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_launch_path),
        launch_arguments={
            'map_path': map_path_3d,
            'param_file_name': param_file_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    ld_points=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(points_launch_path),
        launch_arguments={
            'param_file_name': param_file_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    ld_ndt=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ndt_launch_path),
        launch_arguments={
            'param_file_name': param_file_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    ld_ekf=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path),
        launch_arguments={
            'param_file_name': param_file_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )
    ld_nav2=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'map_path': map_path_2d,
            'param_file_name': param_file_name,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    ld=LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(define_world)
    ld.add_action(define_livox_frame)
    ld.add_action(rviz2)

    ld.add_action(ld_twist)
    ld.add_action(ld_gyro)
    ld.add_action(ld_map)
    ld.add_action(ld_points)
    ld.add_action(ld_ndt)
    ld.add_action(ld_ekf)
    ld.add_action(ld_nav2)

    return ld
