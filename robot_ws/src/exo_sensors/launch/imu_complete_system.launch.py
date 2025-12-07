#!/usr/bin/env python3
"""
启动完整的 IMU 数据采集系统
包括: 读取器 + 记录器 + 可视化器
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('exo_sensors')
    
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='COM3',
        description='串口设备名'
    )
    
    enable_recorder_arg = DeclareLaunchArgument(
        'enable_recorder',
        default_value='true',
        description='是否启用数据记录器'
    )
    
    enable_visualizer_arg = DeclareLaunchArgument(
        'enable_visualizer',
        default_value='true',
        description='是否启用可视化器'
    )
    
    # IMU CSV 读取器
    imu_reader = Node(
        package='exo_sensors',
        executable='imu_csv_reader',
        name='imu_csv_reader',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'imu_csv_reader.yaml'),
            {'serial_port': LaunchConfiguration('serial_port')}
        ]
    )
    
    # IMU 数据记录器
    imu_recorder = Node(
        package='exo_sensors',
        executable='imu_data_recorder',
        name='imu_data_recorder',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'imu_data_recorder.yaml')
        ],
        condition=lambda context: context.launch_configurations['enable_recorder'] == 'true'
    )
    
    # IMU 可视化器
    imu_visualizer = Node(
        package='exo_sensors',
        executable='imu_visualizer',
        name='imu_visualizer',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'imu_visualizer.yaml')
        ],
        condition=lambda context: context.launch_configurations['enable_visualizer'] == 'true'
    )
    
    return LaunchDescription([
        serial_port_arg,
        enable_recorder_arg,
        enable_visualizer_arg,
        imu_reader,
        imu_recorder,
        imu_visualizer
    ])
