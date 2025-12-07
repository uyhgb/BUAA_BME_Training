#!/usr/bin/env python3
"""
数据记录脚本

功能: 订阅IMU和电机命令话题，保存为CSV文件

使用:
    ros2 run robot_controller data_logger.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import csv
from datetime import datetime
import os


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        # 参数
        self.declare_parameter('output_dir', './robot_data')
        self.declare_parameter('log_rate', 10.0)  # Hz
        
        self.output_dir = self.get_parameter('output_dir').value
        self.log_rate = self.get_parameter('log_rate').value
        
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 生成文件名 (带时间戳)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        imu_file = os.path.join(self.output_dir, f'imu_data_{timestamp}.csv')
        motor_file = os.path.join(self.output_dir, f'motor_cmd_{timestamp}.csv')
        
        # 打开CSV文件
        self.imu_file = open(imu_file, 'w', newline='')
        self.motor_file = open(motor_file, 'w', newline='')
        
        # CSV写入器
        self.imu_writer = csv.writer(self.imu_file)
        self.motor_writer = csv.writer(self.motor_file)
        
        # 写入表头
        self.imu_writer.writerow([
            'timestamp', 'roll', 'pitch', 'yaw',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
            'lin_acc_x', 'lin_acc_y', 'lin_acc_z'
        ])
        self.motor_writer.writerow(['timestamp'] + [f'motor_{i}' for i in range(12)])
        
        # 订阅话题
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.motor_sub = self.create_subscription(
            Float32MultiArray, '/motor_commands', self.motor_callback, 10)
        
        self.get_logger().info(f'数据记录器已启动')
        self.get_logger().info(f'IMU数据: {imu_file}')
        self.get_logger().info(f'电机命令: {motor_file}')
        
        # 计数器
        self.imu_count = 0
        self.motor_count = 0
        
    def imu_callback(self, msg):
        """IMU数据回调"""
        # 从四元数计算欧拉角
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )
        
        # 写入CSV
        self.imu_writer.writerow([
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            roll, pitch, yaw,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        self.imu_count += 1
        if self.imu_count % 100 == 0:
            self.get_logger().info(f'已记录 {self.imu_count} 条IMU数据')
    
    def motor_callback(self, msg):
        """电机命令回调"""
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        self.motor_writer.writerow([timestamp] + list(msg.data))
        
        self.motor_count += 1
        if self.motor_count % 50 == 0:
            self.get_logger().info(f'已记录 {self.motor_count} 条电机命令')
    
    def quaternion_to_euler(self, w, x, y, z):
        """四元数转欧拉角"""
        import math
        
        # Roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def __del__(self):
        """析构函数 - 关闭文件"""
        if hasattr(self, 'imu_file'):
            self.imu_file.close()
        if hasattr(self, 'motor_file'):
            self.motor_file.close()
        self.get_logger().info('数据记录器已关闭')


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
