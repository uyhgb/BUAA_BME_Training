#!/usr/bin/env python3
"""
足底传感器状态机启动文件
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
    config_file = os.path.join(pkg_dir, 'config', 'foot_state_machine.yaml')
    
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/foot_usb',
        description='足底传感器串口设备 (固定别名: /dev/foot_usb)'
    )
    
    # 足底状态机节点
    foot_state_node = Node(
        package='exo_sensors',
        executable='foot_state_machine',
        name='foot_state_machine',
        output='screen',
        parameters=[
            config_file,
            {'serial_port': LaunchConfiguration('serial_port')}
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        foot_state_node
    ])
