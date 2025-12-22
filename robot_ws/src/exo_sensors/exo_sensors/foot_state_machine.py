#!/usr/bin/env python3
"""
足底压力传感器状态机节点
功能: 读取足底传感器数据，检测步态状态(落地/抬起)，发布到ROS2话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
import serial
import time
import struct

# --- 核心配置 ---
SERIAL_PORT = '/dev/foot_usb'
BAUD_RATE = 115200
FRAME_HEADER = 0xAA
FRAME_LEN = 39

# 关键点索引 (覆盖脚跟和前掌核心区)
TARGET_INDICES = [0, 1, 2, 3, 6, 7, 8, 9, 12, 13, 16, 17]

# 施密特触发器阈值 (单位: g)
THRESHOLD_ON = 800    
THRESHOLD_OFF = 150   


class GaitDetector:
    """步态检测器 - 使用施密特触发器识别落地/抬起"""
    
    def __init__(self):
        self.is_stance = False  # True=站立期, False=摆动期
        self.baseline = 0       # 零点基准值(去皮)

    def calibrate(self, current_pressure):
        """启动时设置零点，消除传感器底噪"""
        self.baseline = current_pressure

    def process(self, raw_pressure):
        """
        处理压力数据，返回状态变化
        返回: 1=脚落地, 0=脚抬起, -1=状态未改变
        """
        # 计算相对压力 (扣除底噪)
        pressure = max(0, raw_pressure - self.baseline)

        if not self.is_stance:
            if pressure > THRESHOLD_ON:
                self.is_stance = True
                return 1  # 脚落地 (Heel Strike)
        else:
            if pressure < THRESHOLD_OFF:
                self.is_stance = False
                return 0  # 脚抬起 (Toe Off)
        
        return -1  # 状态未改变


class FootStateMachineNode(Node):
    """足底状态机ROS2节点"""
    
    def __init__(self):
        super().__init__('foot_state_machine')
        
        # 声明参数
        self.declare_parameter('serial_port', SERIAL_PORT)
        self.declare_parameter('baud_rate', BAUD_RATE)
        self.declare_parameter('threshold_on', THRESHOLD_ON)
        self.declare_parameter('threshold_off', THRESHOLD_OFF)
        
        # 获取参数
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # 创建发布器
        self.state_publisher = self.create_publisher(
            String, 
            '/foot/gait_state', 
            10
        )
        self.state_code_publisher = self.create_publisher(
            Int32,
            '/foot/state_code',
            10
        )
        self.pressure_publisher = self.create_publisher(
            Float32,
            '/foot/total_pressure',
            10
        )
        
        # 初始化串口
        try:
            self.serial = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.1
            )
            self.serial.reset_input_buffer()
            self.get_logger().info(f'串口已打开: {self.serial_port} @ {self.baud_rate}bps')
        except Exception as e:
            self.get_logger().error(f'串口打开失败: {e}')
            return
        
        # 初始化检测器
        self.detector = GaitDetector()
        self.buffer = bytearray()
        
        # 校准状态
        self.is_calibrated = False
        self.calibration_frames = 0
        self.calibration_sum = 0
        
        # 设备信息
        self.device_id = None
        self.foot_name = "未知"
        
        # 创建定时器读取数据
        self.create_timer(0.005, self.read_serial_data)  # 5ms 查询间隔
        
        self.get_logger().info('足底状态机节点已启动，正在校准零点...')
    
    def verify_checksum(self, data):
        """验证校验和 (前38字节之和的低八位)"""
        return (sum(data[:38]) & 0xFF) == data[38]
    
    def parse_sensor_data(self, payload):
        """解析传感器数据 (高位在前，低位在后)"""
        vals = []
        for i in range(0, 36, 2):
            val = (payload[i] << 8) | payload[i+1]
            vals.append(val)
        return vals
    
    def read_serial_data(self):
        """定时读取串口数据"""
        if not hasattr(self, 'serial') or not self.serial.is_open:
            return
        
        try:
            # 读取可用数据
            if self.serial.in_waiting:
                self.buffer.extend(self.serial.read(self.serial.in_waiting))
                
                # 安全机制：防止缓冲区无限膨胀
                if len(self.buffer) > 200:
                    self.buffer = self.buffer[-200:]
            
            # 处理完整帧
            while len(self.buffer) >= FRAME_LEN:
                # 查找帧头
                if self.buffer[0] != FRAME_HEADER:
                    self.buffer.pop(0)
                    continue
                
                frame = self.buffer[:FRAME_LEN]
                
                # 验证校验和
                if self.verify_checksum(frame):
                    # 获取设备ID (01=左脚, 02=右脚)
                    self.device_id = frame[1]
                    self.foot_name = "左脚" if self.device_id == 0x01 else "右脚"
                    
                    # 解析数据
                    payload = frame[2:38]
                    pressures = self.parse_sensor_data(payload)
                    
                    # 计算目标区域总压力
                    target_pressures = [pressures[i] for i in TARGET_INDICES]
                    total_force = sum(target_pressures)
                    
                    # 处理数据
                    self.process_pressure_data(total_force)
                    
                    self.buffer = self.buffer[FRAME_LEN:]
                else:
                    self.buffer.pop(0)
        
        except Exception as e:
            self.get_logger().error(f'读取串口数据时出错: {e}')
    
    def process_pressure_data(self, total_force):
        """处理压力数据并发布状态"""
        
        # --- 校准阶段 ---
        if not self.is_calibrated:
            self.calibration_sum += total_force
            self.calibration_frames += 1
            
            if self.calibration_frames >= 10:  # 取前10帧平均值
                avg_baseline = int(self.calibration_sum / 10)
                self.detector.calibrate(avg_baseline)
                self.is_calibrated = True
                self.get_logger().info(
                    f'校准完成 ({self.foot_name}) | 基准值: {avg_baseline}g'
                )
            return
        
        # --- 正常检测阶段 ---
        state_change = self.detector.process(total_force)
        
        # 发布压力数据
        pressure_msg = Float32()
        pressure_msg.data = float(total_force - self.detector.baseline)
        self.pressure_publisher.publish(pressure_msg)
        
        # 状态改变时发布
        if state_change != -1:
            # 发布状态字符串
            state_msg = String()
            if state_change == 1:
                state_msg.data = "stance"  # 站立期 (脚落地)
                log_msg = f'▼ 脚落地 (Heel Strike) | 净压力: {total_force - self.detector.baseline}g'
            else:
                state_msg.data = "swing"   # 摆动期 (脚抬起)
                log_msg = f'▲ 脚抬起 (Toe Off) | 净压力: {total_force - self.detector.baseline}g'
            
            self.state_publisher.publish(state_msg)
            
            # 发布状态码
            code_msg = Int32()
            code_msg.data = state_change
            self.state_code_publisher.publish(code_msg)
            
            # 打印日志
            self.get_logger().info(f'{self.foot_name} {log_msg}')
    
    def __del__(self):
        """节点销毁时关闭串口"""
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
            self.get_logger().info('串口已关闭')


def main(args=None):
    rclpy.init(args=args)
    node = FootStateMachineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
