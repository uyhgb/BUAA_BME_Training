#!/usr/bin/env python3
"""
IMU CSV 读取器 - 开发环境启动文件
使用场景: Windows + WSL2 + Docker, 通过 usbipd-win 连接 USB 串口
ESP32 输出 CSV 格式: Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('exo_sensors')
    
    # 配置文件路径 - 使用 WSL2 开发环境配置
    config_file = os.path.join(pkg_share, 'config', 'imu_csv_dev_wsl.yaml')
    
    # 声明启动参数 (可以覆盖配置文件)
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/imu_usb',  # IMU固定别名
        description='Serial port device path (固定别名: /dev/imu_usb)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
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
        ],
        # 添加环境变量以便调试
        additional_env={'PYTHONUNBUFFERED': '1'}
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        imu_reader_node
    ])
