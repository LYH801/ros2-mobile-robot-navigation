#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home = os.path.expanduser('~')

    # ========== 关键：设置 GAZEBO_MODEL_PATH ==========
    gazebo_models_path = os.path.join(
        home, 'robotics_study/gazebo_models')
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_models_path + ':' +
              os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    # 1. 启动 Gazebo 世界
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            )
        ),
    )

    # 2. spawn ArUco 标记（延迟 5 秒）
    spawn_marker = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-database', 'aruco_marker',
                    '-entity', 'aruco_marker_0',
                    '-x', '1.0', '-y', '-0.5', '-z', '0',
                    '-Y', '3.14159',
                ],
                output='screen',
            )
        ]
    )

    # 3. 启动 Nav2（延迟 10 秒）
    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('turtlebot3_navigation2'),
                        'launch',
                        'navigation2.launch.py'
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'map': os.path.join(
                        home, 'robotics_ws/maps/training_map.yaml'),
                    'params_file': os.path.join(
                        home,
                        'robotics_ws/install/pursuit_controller/share/',
                        'pursuit_controller/config/nav2_params_pursuit.yaml'
                    ),
                }.items(),
            )
        ]
    )

    return LaunchDescription([
        set_gazebo_model_path,   # 关键：先设置环境变量
        gazebo_launch,
        spawn_marker,
        nav2_launch,
    ])
