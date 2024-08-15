# SPDX-FileCopyrightText: 2024 nacky823 youjiyongmu4@gmail.com
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    home_path=os.path.expanduser('~')
    costmap_path=os.path.join(home_path, 'maps', 'iscas_museum', 'iscas_museum_2d.yaml')

    rviz_config_file=os.path.join(
        get_package_share_directory('mugimaru_launcher'),
        'config',
        'sim_iscas.rviz',
    )
    rviz2=Node(
        package='rviz2',
        executable='rviz2',
        name='mugimaru_rviz2',
        namespace='',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
        prefix='xterm -e',
    )

    world_to_map=Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
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
        cmd=[ 'ros2', 'launch', 'ros2_odometry_twist_converter', 'odom_to_twist_cov_stamp.launch.py' ],
        output='screen',
    )
    exec_gyro=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'gyro_odometer', 'nacky_gyro_odometer.launch.xml' ],
        output='screen',
    )
    exec_map=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'map_loader', 'nacky_test.launch.py' ],
        output='screen',
    )
    exec_points=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'pointcloud_preprocessor', 'nacky_test.launch.py' ],
        output='screen',
    )
    exec_ndt=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'ndt_scan_matcher', 'nacky_test.launch.py' ],
        output='screen',
    )
    exec_ekf=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'ekf_localizer', 'ekf_localizer.launch.xml' ],
        output='screen',
    )
    exec_nav2=ExecuteProcess(
        cmd=[ 'ros2', 'launch', 'raspicat_navigation', 'raspicat_nav2.launch.py', f'map:={costmap_path}' ],
        output='screen',
        prefix='xterm -e',
    )

    ld=LaunchDescription()

    ld.add_action(rviz2)
    ld.add_action(world_to_map)
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
