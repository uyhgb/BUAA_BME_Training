#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机驱动节点
功能: 订阅力矩指令 -> 调用Unitree SDK -> 驱动电机
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import sys
import time

# 引入宇树 SDK
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
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/unitree_motor')
        self.declare_parameter('max_torque', 23.0)
        
        self.SERIAL_PORT = self.get_parameter('serial_port').value
        self.MAX_TORQUE = self.get_parameter('max_torque').value
        
        # 初始化 SDK 对象
        self.sdk_ready = False
        if SDK_AVAILABLE:
            try:
                self.serial = SerialPort(self.SERIAL_PORT)
                self.cmd = MotorCmd()
                self.data = MotorData()
                self.cmd.motorType = MotorType.GO_M8010_6
                self.data.motorType = MotorType.GO_M8010_6
                self.sdk_ready = True
                
                # 激活电机
                self.cmd.mode = 1
                self.cmd.kp = 0
                self.cmd.kd = 1.0
                self.cmd.tau = 0
                self.serial.sendRecv(self.cmd, self.data)
                self.get_logger().info('✅ 电机连接成功')
            except Exception as e:
                self.get_logger().error(f'❌ 电机连接失败: {e}')
                self.sdk_ready = False
        
        if not self.sdk_ready:
            self.get_logger().warn('⚠️ 无法连接电机，运行在模拟模式')

        # 订阅力矩指令
        self.sub_cmd = self.create_subscription(
            Float32, 
            '/control/cmd_torque', 
            self.cmd_callback, 
            10
        )

    def cmd_callback(self, msg):
        """力矩指令回调"""
        target_torque = msg.data
        
        # 安全限幅
        target_torque = max(min(target_torque, self.MAX_TORQUE), -self.MAX_TORQUE)

        if self.sdk_ready:
            self.cmd.mode = 1           # FOC 模式
            self.cmd.kp = 0.0           # 刚度=0 (消音)
            self.cmd.kd = 1.0           # 阻尼=1 (防飞车)
            self.cmd.q = 0.0
            self.cmd.dq = 0.0
            self.cmd.tau = target_torque
            
            self.serial.sendRecv(self.cmd, self.data)
            self.get_logger().debug(f'电机力矩: {target_torque:.2f} Nm')
        else:
            # 模拟模式：仅打印
            if abs(target_torque) > 0.1:
                self.get_logger().info(f'虚拟驱动: {target_torque:.2f} Nm')

    def destroy_node(self):
        """退出前刹车"""
        if self.sdk_ready:
            self.cmd.tau = 0
            self.cmd.kd = 2.0
            self.serial.sendRecv(self.cmd, self.data)
            self.get_logger().info('电机已刹车')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()