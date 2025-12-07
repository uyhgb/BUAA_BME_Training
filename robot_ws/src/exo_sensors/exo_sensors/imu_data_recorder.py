#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 数据记录器
将 IMU 数据保存为 CSV 格式，用于 SVM 步态识别训练
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import os
from datetime import datetime
import math


class IMUDataRecorder(Node):
    """IMU 数据记录节点"""
    
    def __init__(self):
        super().__init__('imu_data_recorder')
        
        # 声明参数
        self.declare_parameter('topic_name', '/imu/data')
        self.declare_parameter('output_dir', './data')
        self.declare_parameter('file_prefix', 'imu_data')
        self.declare_parameter('auto_filename', True)
        
        # 获取参数
        self.topic_name = self.get_parameter('topic_name').value
        self.output_dir = self.get_parameter('output_dir').value
        self.file_prefix = self.get_parameter('file_prefix').value
        self.auto_filename = self.get_parameter('auto_filename').value
        
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 生成文件名
        if self.auto_filename:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'{self.file_prefix}_{timestamp}.csv'
        else:
            filename = f'{self.file_prefix}.csv'
        
        self.csv_path = os.path.join(self.output_dir, filename)
        
        # 打开 CSV 文件
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        
        # 写入表头
        self.csv_writer.writerow([
            'Timestamp', 'Roll', 'Pitch', 'Yaw',
            'AccX', 'AccY', 'AccZ',
            'GyroX', 'GyroY', 'GyroZ'
        ])
        
        # 创建订阅器
        self.subscription = self.create_subscription(
            Imu,
            self.topic_name,
            self.imu_callback,
            10
        )
        
        self.sample_count = 0
        self.start_time = self.get_clock().now()
        
        self.get_logger().info(f'IMU 数据记录器已启动')
        self.get_logger().info(f'数据保存至: {self.csv_path}')
    
    def quaternion_to_euler(self, qx, qy, qz, qw):
        """
        四元数转欧拉角
        返回: (roll, pitch, yaw) 弧度
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def imu_callback(self, msg):
        """IMU 数据回调"""
        try:
            # 提取时间戳 (转换为毫秒)
            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            timestamp_ms = timestamp_sec * 1000 + timestamp_nanosec // 1000000
            
            # 四元数转欧拉角
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            
            roll_rad, pitch_rad, yaw_rad = self.quaternion_to_euler(qx, qy, qz, qw)
            
            # 弧度转角度
            roll_deg = math.degrees(roll_rad)
            pitch_deg = math.degrees(pitch_rad)
            yaw_deg = math.degrees(yaw_rad)
            
            # 线性加速度 (m/s² → mg)
            acc_x_mg = msg.linear_acceleration.x / 0.00980665
            acc_y_mg = msg.linear_acceleration.y / 0.00980665
            acc_z_mg = msg.linear_acceleration.z / 0.00980665
            
            # 角速度 (rad/s → dps)
            gyro_x_dps = math.degrees(msg.angular_velocity.x)
            gyro_y_dps = math.degrees(msg.angular_velocity.y)
            gyro_z_dps = math.degrees(msg.angular_velocity.z)
            
            # 写入 CSV
            self.csv_writer.writerow([
                timestamp_ms,
                f'{roll_deg:.2f}',
                f'{pitch_deg:.2f}',
                f'{yaw_deg:.2f}',
                f'{acc_x_mg:.3f}',
                f'{acc_y_mg:.3f}',
                f'{acc_z_mg:.3f}',
                f'{gyro_x_dps:.2f}',
                f'{gyro_y_dps:.2f}',
                f'{gyro_z_dps:.2f}'
            ])
            
            self.sample_count += 1
            
            # 每 100 个样本打印一次
            if self.sample_count % 100 == 0:
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                rate = self.sample_count / elapsed if elapsed > 0 else 0
                self.get_logger().info(f'已记录 {self.sample_count} 个样本 ({rate:.1f} Hz)')
                
        except Exception as e:
            self.get_logger().error(f'记录数据时出错: {e}')
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
            self.get_logger().info(f'数据已保存: {self.csv_path} ({self.sample_count} 个样本)')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUDataRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
