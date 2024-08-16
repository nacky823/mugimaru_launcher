# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time=LaunchConfiguration('use_sim_time')
    declare_use_sim_time=DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
    )

    rviz_config_file=os.path.join(
        get_package_share_directory('mugimaru_launcher'),
        'config', 'iscas_museum.rviz'
    )
    rviz2=Node(
        package='rviz2',
        executable='rviz2',
        name='mugimaru_rviz2',
        namespace='',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        prefix='xterm -e',
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
        output='screen',
    )

    exec_gazebo=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'raspicat_gazebo', 'raspicat_with_iscas_museum.launch.py', 'rviz:=false' ],
        output='screen',
    )
    exec_twist=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'ros2_odometry_twist_converter', 'mugimaru_sim.launch.py' ],
        output='screen',
    )
    exec_gyro=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'gyro_odometer', 'sim_iscas.launch.xml' ],
        output='screen',
    )
    exec_map=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'map_loader', 'sim_iscas.launch.py' ],
        output='screen',
    )
    exec_points=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'pointcloud_preprocessor', 'sim_iscas.launch.py' ],
        output='screen',
    )
    exec_ndt=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'ndt_scan_matcher', 'sim_iscas.launch.py' ],
        output='screen',
    )
    exec_ekf=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'ekf_localizer', 'sim_iscas.launch.xml' ],
        output='screen',
    )
    exec_nav2=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'mugimaru_navigation2', 'iscas_museum.launch.py' ],
        output='screen',
    )

    ld=LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(rviz2)
    ld.add_action(define_world)
    ld.add_action(define_livox_frame)

    ld.add_action(exec_gazebo)
    ld.add_action(exec_twist)
    ld.add_action(exec_gyro)
    ld.add_action(exec_map)
    ld.add_action(exec_points)
    ld.add_action(exec_ndt)
    ld.add_action(exec_ekf)
    ld.add_action(exec_nav2)

    return ld
