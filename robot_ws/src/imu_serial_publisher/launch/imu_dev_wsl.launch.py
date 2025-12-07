"""
IMU 串口发布器 - 开发环境启动文件
使用场景: Windows + WSL2 + Docker, 通过 usbipd-win 连接 USB 串口
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('imu_serial_publisher')
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'imu_params_dev_wsl.yaml')
    
    # 声明启动参数 (可以覆盖配置文件)
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port device path (WSL2 usbipd-win)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )
    
    # IMU串口发布节点
    imu_publisher_node = Node(
        package='imu_serial_publisher',
        executable='imu_serial_publisher_node',
        name='imu_serial_publisher',
        output='screen',
        parameters=[config_file, {
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate')
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        imu_publisher_node
    ])
