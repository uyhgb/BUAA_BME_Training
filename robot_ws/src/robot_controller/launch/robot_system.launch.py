from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('robot_controller'),
        'config'
    )
    controller_params = os.path.join(config_dir, 'controller_params.yaml')
    
    # 声明启动参数
    enable_control_arg = DeclareLaunchArgument(
        'enable_control',
        default_value='false',
        description='Enable motor control (safety switch)'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for IMU'
    )
    
    # IMU串口发布节点
    imu_publisher_node = Node(
        package='imu_serial_publisher',
        executable='imu_serial_publisher_node',
        name='imu_publisher',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 115200,
            'frame_id': 'imu_link',
            'publish_rate': 50.0
        }]
    )
    
    # 机器人控制器节点
    controller_node = Node(
        package='robot_controller',
        executable='main_controller',
        name='robot_controller',
        output='screen',
        parameters=[
            controller_params,
            {
                'enable_control': LaunchConfiguration('enable_control')
            }
        ]
    )
    
    return LaunchDescription([
        enable_control_arg,
        serial_port_arg,
        imu_publisher_node,
        controller_node
    ])
