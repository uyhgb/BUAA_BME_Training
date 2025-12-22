#!/usr/bin/env python3
"""
外骨骼控制系统完整启动文件
启动顺序: IMU读取器 -> 步态推理 -> 电机驱动
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    exoskel_pkg_dir = get_package_share_directory('exoskel_control_py')
    exo_sensors_pkg_dir = get_package_share_directory('exo_sensors')
    
    # 配置文件路径
    gait_config = os.path.join(exoskel_pkg_dir, 'config', 'gait_inference.yaml')
    motor_config = os.path.join(exoskel_pkg_dir, 'config', 'motor_driver.yaml')
    imu_config = os.path.join(exo_sensors_pkg_dir, 'config', 'imu_csv_raspberry_pi.yaml')
    
    # 声明启动参数
    enable_motor_arg = DeclareLaunchArgument(
        'enable_motor',
        default_value='false',
        description='是否启用电机驱动 (安全开关)'
    )
    
    # 1. IMU数据读取节点
    imu_reader_node = Node(
        package='exo_sensors',
        executable='imu_csv_reader',
        name='imu_csv_reader',
        output='screen',
        parameters=[imu_config]
    )
    
    # 2. 步态推理节点
    gait_inference_node = Node(
        package='exoskel_control_py',
        executable='gait_inference_node',
        name='gait_inference_node',
        output='screen',
        parameters=[gait_config]
    )
    
    # 3. 电机驱动节点 (可选)
    motor_driver_node = Node(
        package='exoskel_control_py',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen',
        parameters=[motor_config],
        condition=IfCondition(LaunchConfiguration('enable_motor'))
    )
    
    return LaunchDescription([
        enable_motor_arg,
        imu_reader_node,
        gait_inference_node,
        motor_driver_node
    ])
