#!/usr/bin/env python3
"""
实时数据可视化脚本

功能: 订阅IMU话题，实时绘制姿态曲线

依赖:
    pip install matplotlib

使用:
    ros2 run robot_controller visualization.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import math


class IMUVisualizer(Node):
    def __init__(self):
        super().__init__('imu_visualizer')
        
        # 参数
        self.declare_parameter('buffer_size', 200)
        buffer_size = self.get_parameter('buffer_size').value
        
        # 数据缓冲区
        self.time_data = deque(maxlen=buffer_size)
        self.roll_data = deque(maxlen=buffer_size)
        self.pitch_data = deque(maxlen=buffer_size)
        self.yaw_data = deque(maxlen=buffer_size)
        
        # 订阅IMU话题
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # 起始时间
        self.start_time = None
        
        # 设置matplotlib
        self.fig, self.axes = plt.subplots(3, 1, figsize=(10, 8))
        self.fig.suptitle('IMU实时姿态数据')
        
        # 配置子图
        labels = ['Roll (°)', 'Pitch (°)', 'Yaw (°)']
        colors = ['r', 'g', 'b']
        
        self.lines = []
        for ax, label, color in zip(self.axes, labels, colors):
            ax.set_ylabel(label)
            ax.set_xlabel('Time (s)')
            ax.grid(True)
            line, = ax.plot([], [], color, linewidth=2)
            self.lines.append(line)
        
        self.get_logger().info('IMU可视化器已启动')
        self.get_logger().info('关闭matplotlib窗口停止程序')
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        # 设置起始时间
        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # 当前时间
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        elapsed = current_time - self.start_time
        
        # 从四元数计算欧拉角
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        )
        
        # 转换为度
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # 添加到缓冲区
        self.time_data.append(elapsed)
        self.roll_data.append(roll_deg)
        self.pitch_data.append(pitch_deg)
        self.yaw_data.append(yaw_deg)
    
    def update_plot(self, frame):
        """更新绘图"""
        if len(self.time_data) == 0:
            return self.lines
        
        time_list = list(self.time_data)
        
        # 更新Roll
        self.lines[0].set_data(time_list, list(self.roll_data))
        self.axes[0].relim()
        self.axes[0].autoscale_view()
        
        # 更新Pitch
        self.lines[1].set_data(time_list, list(self.pitch_data))
        self.axes[1].relim()
        self.axes[1].autoscale_view()
        
        # 更新Yaw
        self.lines[2].set_data(time_list, list(self.yaw_data))
        self.axes[2].relim()
        self.axes[2].autoscale_view()
        
        return self.lines
    
    def quaternion_to_euler(self, w, x, y, z):
        """四元数转欧拉角"""
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
    
    def show(self):
        """显示窗口"""
        ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=50, blit=True)
        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = IMUVisualizer()
    
    # 在单独的线程中spin ROS2
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        # 显示matplotlib窗口 (阻塞)
        node.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
