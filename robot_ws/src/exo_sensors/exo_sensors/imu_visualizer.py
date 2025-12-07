#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU 数据可视化器
实时绘制 IMU 姿态角度
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation


class IMUVisualizer(Node):
    """IMU 数据可视化节点"""
    
    def __init__(self):
        super().__init__('imu_visualizer')
        
        # 声明参数
        self.declare_parameter('topic_name', '/imu/data')
        self.declare_parameter('window_size', 200)
        
        # 获取参数
        self.topic_name = self.get_parameter('topic_name').value
        self.window_size = self.get_parameter('window_size').value
        
        # 数据缓冲区
        self.time_data = deque(maxlen=self.window_size)
        self.roll_data = deque(maxlen=self.window_size)
        self.pitch_data = deque(maxlen=self.window_size)
        self.yaw_data = deque(maxlen=self.window_size)
        
        self.start_time = None
        
        # 创建订阅器
        self.subscription = self.create_subscription(
            Imu,
            self.topic_name,
            self.imu_callback,
            10
        )
        
        # 设置 matplotlib
        plt.style.use('seaborn-v0_8-darkgrid')
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 8))
        self.fig.suptitle('IMU 姿态角实时监控', fontsize=16, fontweight='bold')
        
        # 配置子图
        titles = ['Roll (横滚角)', 'Pitch (俯仰角)', 'Yaw (偏航角)']
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c']
        
        self.lines = []
        for i, (ax, title, color) in enumerate(zip(self.axes, titles, colors)):
            line, = ax.plot([], [], color=color, linewidth=2, label=title)
            ax.set_ylabel('角度 (°)', fontsize=12)
            ax.set_ylim(-180, 180)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            self.lines.append(line)
        
        self.axes[-1].set_xlabel('时间 (秒)', fontsize=12)
        
        plt.tight_layout()
        
        self.get_logger().info('IMU 可视化器已启动')
    
    def quaternion_to_euler(self, qx, qy, qz, qw):
        """四元数转欧拉角"""
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def imu_callback(self, msg):
        """IMU 数据回调"""
        try:
            # 初始化起始时间
            if self.start_time is None:
                self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            
            # 计算相对时间
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            elapsed_time = current_time - self.start_time
            
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
            
            # 添加到缓冲区
            self.time_data.append(elapsed_time)
            self.roll_data.append(roll_deg)
            self.pitch_data.append(pitch_deg)
            self.yaw_data.append(yaw_deg)
            
        except Exception as e:
            self.get_logger().error(f'处理数据时出错: {e}')
    
    def update_plot(self, frame):
        """更新绘图"""
        if len(self.time_data) > 0:
            time_list = list(self.time_data)
            
            # 更新 Roll
            self.lines[0].set_data(time_list, list(self.roll_data))
            self.axes[0].set_xlim(max(0, time_list[-1] - 10), time_list[-1] + 0.5)
            
            # 更新 Pitch
            self.lines[1].set_data(time_list, list(self.pitch_data))
            self.axes[1].set_xlim(max(0, time_list[-1] - 10), time_list[-1] + 0.5)
            
            # 更新 Yaw
            self.lines[2].set_data(time_list, list(self.yaw_data))
            self.axes[2].set_xlim(max(0, time_list[-1] - 10), time_list[-1] + 0.5)
        
        return self.lines
    
    def run(self):
        """运行可视化"""
        # 创建动画
        ani = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            interval=50,  # 20 Hz 更新
            blit=True
        )
        
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = IMUVisualizer()
    
    try:
        # 在单独线程中运行 ROS2
        import threading
        ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
        ros_thread.start()
        
        # 主线程运行 matplotlib
        node.run()
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
