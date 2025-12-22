#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU CSV 数据读取器
从 ESP32 串口读取 CSV 格式的 IMU 数据并发布到 ROS2 话题
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import serial
import serial.tools.list_ports
import math
import time


class IMUCSVReader(Node):
    """IMU CSV 数据读取节点"""
    
    def __init__(self):
        super().__init__('imu_csv_reader')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/imu_usb')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('topic_name', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('auto_detect_port', False)
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.topic_name = self.get_parameter('topic_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.auto_detect = self.get_parameter('auto_detect_port').value
        
        # 创建发布器
        self.imu_publisher = self.create_publisher(Imu, self.topic_name, 10)
        
        # 自动检测串口
        if self.auto_detect:
            self.serial_port = self.detect_esp32_port()
            if not self.serial_port:
                self.get_logger().error('无法自动检测到 ESP32 串口设备')
                return
        
        # 初始化串口
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            self.get_logger().info(f'串口已打开: {self.serial_port} @ {self.baud_rate}bps')
            
            # 等待串口稳定
            time.sleep(2)
            self.serial.reset_input_buffer()
            
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口 {self.serial_port}: {e}')
            return
        
        # 统计变量
        self.packet_count = 0
        self.error_count = 0
        self.last_log_time = time.time()
        
        # 创建定时器读取数据
        self.create_timer(0.001, self.read_serial_data)  # 1ms 检查一次
        
        self.get_logger().info('IMU CSV 读取器已启动，等待数据...')
    
    def detect_esp32_port(self):
        """自动检测 ESP32 串口"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # ESP32 常见的 USB 转串口芯片
            description = port.description or ''
            manufacturer = port.manufacturer or ''
            
            if 'CP210' in description or 'CH340' in description or \
               'USB-SERIAL' in description or 'Silicon Labs' in manufacturer:
                self.get_logger().info(f'检测到 ESP32: {port.device} - {description}')
                return port.device
        return None
    
    def read_serial_data(self):
        """读取串口数据"""
        if not hasattr(self, 'serial') or not self.serial.is_open:
            return
        
        try:
            if self.serial.in_waiting > 0:
                line = self.serial.readline().decode('utf-8').strip()
                
                # 过滤掉单位行和空行
                if not line or line in ['mg', 'dps', 'm/s2', 'rad/s', 'Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ']:
                    return
                
                # 解析 CSV 数据
                self.parse_and_publish(line)
                
        except UnicodeDecodeError:
            self.error_count += 1
            self.get_logger().warn('数据解码错误，跳过此行')
        except serial.SerialException as e:
            # 串口硬件错误（断连、多路访问等）
            self.error_count += 1
            if 'device disconnected' in str(e) or 'multiple access' in str(e):
                self.get_logger().warn(f'串口瞬时异常（可能是USB供电不足）: {e}')
            else:
                self.get_logger().error(f'串口异常: {e}')
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'读取串口数据时出错: {e}')
    
    def parse_and_publish(self, line):
        """解析 CSV 数据并发布"""
        try:
            # CSV 格式: Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ
            parts = line.split(',')
            
            if len(parts) != 10:
                self.error_count += 1
                return
            
            # 解析数据
            timestamp = int(parts[0])
            roll = float(parts[1])    # 度
            pitch = float(parts[2])   # 度
            yaw = float(parts[3])     # 度
            acc_x = float(parts[4])   # mg
            acc_y = float(parts[5])   # mg
            acc_z = float(parts[6])   # mg
            gyro_x = float(parts[7])  # dps
            gyro_y = float(parts[8])  # dps
            gyro_z = float(parts[9])  # dps
            
            # 单位转换
            # 角度 → 弧度 (ROS 标准)
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)
            yaw_rad = math.radians(yaw)
            
            # mg → m/s² (ROS 标准)
            acc_x_ms2 = acc_x * 0.00980665  # 1mg = 0.00980665 m/s²
            acc_y_ms2 = acc_y * 0.00980665
            acc_z_ms2 = acc_z * 0.00980665
            
            # dps → rad/s (ROS 标准)
            gyro_x_rads = math.radians(gyro_x)
            gyro_y_rads = math.radians(gyro_y)
            gyro_z_rads = math.radians(gyro_z)
            
            # 欧拉角 → 四元数
            quaternion = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
            
            # 构造 IMU 消息
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            
            # 姿态 (四元数)
            msg.orientation.x = quaternion[0]
            msg.orientation.y = quaternion[1]
            msg.orientation.z = quaternion[2]
            msg.orientation.w = quaternion[3]
            
            # 角速度 (rad/s)
            msg.angular_velocity.x = gyro_x_rads
            msg.angular_velocity.y = gyro_y_rads
            msg.angular_velocity.z = gyro_z_rads
            
            # 线性加速度 (m/s²)
            msg.linear_acceleration.x = acc_x_ms2
            msg.linear_acceleration.y = acc_y_ms2
            msg.linear_acceleration.z = acc_z_ms2
            
            # 发布消息
            self.imu_publisher.publish(msg)
            
            self.packet_count += 1
            
            # 每秒打印一次统计
            current_time = time.time()
            if current_time - self.last_log_time >= 1.0:
                self.get_logger().info(
                    f'[统计] 成功: {self.packet_count} 包/秒, 错误: {self.error_count} 包/秒 | '
                    f'姿态: R={roll:.1f}° P={pitch:.1f}° Y={yaw:.1f}°'
                )
                self.packet_count = 0
                self.error_count = 0
                self.last_log_time = current_time
                
        except ValueError as e:
            self.error_count += 1
            self.get_logger().warn(f'数据解析错误: {line} ({e})')
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'处理数据时出错: {e}')
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        欧拉角转四元数
        roll, pitch, yaw: 弧度
        返回: [qx, qy, qz, qw]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return [qx, qy, qz, qw]
    
    def destroy_node(self):
        """清理资源"""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUCSVReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
