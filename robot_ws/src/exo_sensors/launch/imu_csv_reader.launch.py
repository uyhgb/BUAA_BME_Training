#!/usr/bin/env python3
"""
启动 IMU CSV 读取器
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
    config_file = os.path.join(pkg_dir, 'config', 'imu_csv_reader.yaml')
    
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/imu_usb',
        description='串口设备名 (固定别名: /dev/imu_usb)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='串口波特率'
    )
    
    # IMU CSV 读取器节点
    imu_reader_node = Node(
        package='exo_sensors',
        executable='imu_csv_reader',
        name='imu_csv_reader',
        output='screen',
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
            }
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        imu_reader_node
    ])
