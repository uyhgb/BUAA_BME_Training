#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机驱动节点 (柔顺控制 & 高频通信版)
功能: 
1. 1000Hz 高频维持电机通信 (心跳)
2. 接收 SVM 力矩指令并执行
3. 采用低阻抗参数 (Kp=0, Kd=0.05) 实现静音与透明感
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
import sys
import time

# 引入 SDK
sys.path.append('../lib')
try:
    from unitree_actuator_sdk import *
    SDK_AVAILABLE = True
except ImportError:
    print("⚠️ Unitree SDK 缺失，进入模拟模式")
    SDK_AVAILABLE = False

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # --- 1. 参数配置 ---
        self.declare_parameter('serial_port', '/dev/unitree_motor')
        self.SERIAL_PORT = self.get_parameter('serial_port').value
        
        # 柔顺控制参数 (源自之前的静音测试)
        self.TARGET_KP = 0.0   # 必须为0！否则会像皮筋一样把人拉回0度
        self.TARGET_KD = 0.05  # 0.05 是之前测试出的最佳静音阻尼
        self.MAX_TORQUE = 23.0 # 物理保护限幅
        
        # --- 2. 初始化 SDK ---
        self.sdk_ready = False
        if SDK_AVAILABLE:
            try:
                self.serial = SerialPort(self.SERIAL_PORT)
                self.cmd = MotorCmd()
                self.data = MotorData()
                self.cmd.motorType = MotorType.GO_M8010_6
                self.data.motorType = MotorType.GO_M8010_6
                self.sdk_ready = True
                
                # 激活握手
                self.cmd.mode = 1
                self.cmd.kp = 0.0
                self.cmd.kd = 0.0
                self.cmd.tau = 0.0
                self.serial.sendRecv(self.cmd, self.data)
                self.get_logger().info(f'✅ 电机连接成功 Port: {self.SERIAL_PORT}')
            except Exception as e:
                self.get_logger().error(f'❌ 连接失败: {e}')
        
        # --- 3. 状态变量 ---
        self.current_target_torque = 0.0
        
        # --- 4. 通信机制 (核心修改) ---
        # 订阅指令 (只负责更新变量，不负责发串口)
        self.sub_cmd = self.create_subscription(
            Float32, '/control/cmd_torque', self.cmd_callback, 10
        )
        
        # 发布电机状态 (SVM 可能需要这个反馈: [q, dq, tau])
        self.pub_state = self.create_publisher(Float32MultiArray, '/motor/state', 10)

        # 创建 1000Hz 定时器 (1ms) -> 替代之前的 while True
        # 这保证了无论 ROS 消息发得多慢，电机通信永远流畅
        self.create_timer(0.001, self.timer_callback)

    def cmd_callback(self, msg):
        """收到 SVM 指令，仅更新目标变量"""
        raw_torque = msg.data
        # 安全限幅
        self.current_target_torque = max(min(raw_torque, self.MAX_TORQUE), -self.MAX_TORQUE)

    def timer_callback(self):
        """1000Hz 实时控制循环 (核心)"""
        if not self.sdk_ready:
            return

        # 1. 设置指令
        self.cmd.mode = 1           # FOC 闭环
        self.cmd.q = 0.0            # 位置设为0 (配合Kp=0使用)
        self.cmd.dq = 0.0           # 速度设为0 (配合Kd=0.05使用)
        
        # 关键: 使用静音参数
        self.cmd.kp = self.TARGET_KP  # 0.0 (被动模式)
        self.cmd.kd = self.TARGET_KD  # 0.05 (丝滑阻尼)
        self.cmd.tau = self.current_target_torque # SVM 计算出的力矩

        # 2. 发送与接收 (USB 3.0 独享带宽下，这会非常快)
        self.serial.sendRecv(self.cmd, self.data)

        # 3. 发布状态给 ROS (SVM需要知道当前电机角度来做下一步推理)
        # 格式: [角度, 速度, 实际力矩]
        state_msg = Float32MultiArray()
        # 注意: 这里假设 data.q 是转子端，如果SVM需要关节端，请除以 6.33
        state_msg.data = [self.data.q, self.data.dq, self.data.tau]
        self.pub_state.publish(state_msg)

    def destroy_node(self):
        """安全退出"""
        if self.sdk_ready:
            self.cmd.mode = 1
            self.cmd.tau = 0.0
            self.cmd.kd = 1.0 # 退出时稍微加点阻尼防止垂落
            for _ in range(5):
                self.serial.sendRecv(self.cmd, self.data)
                time.sleep(0.002)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        # rclpy.spin 会自动管理 timer_callback 的 1000Hz 调度
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()