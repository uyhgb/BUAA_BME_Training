#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步态推理节点 (最终安全版)
功能: 
1. 接收 IMU 数据 -> SVM 预测
2. 【安全锁1】静止检测 (方差阈值)
3. 【安全锁2】相位截断 (强制 0~45% 区间 0 力矩) <--- 关键补充
4. 平滑滤波输出
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import joblib
import numpy as np
import math
from collections import deque
from scipy.signal import detrend
import os
from ament_index_python.packages import get_package_share_directory

class GaitInferenceNode(Node):
    def __init__(self):
        super().__init__('gait_inference_node')
        
        self.get_logger().info('🛡️ 步态推理节点 (安全增强版) 启动...')
        self.count = 0
        
        # 1. 路径配置
        try:
            pkg_dir = get_package_share_directory('exoskel_control_py')
        except Exception:
            pkg_dir = ''
        
        self.declare_parameter('model_path', os.path.join(pkg_dir, 'config'))
        data_dir = self.get_parameter('model_path').value
        
        # 2. 助力参数
        self.ASSIST_PEAK_PHASE = 0.60
        self.PHASE_LEAD = 0.07  #观察到输出基本是0.53相位处
        self.ASSIST_WIDTH = 0.25   # 高斯宽度，控制助力曲线陡峭度，就是sigma
        self.MAX_TORQUE = 23.0  # 与motor_driver_node保持一致
        self.CENTER_PHASE = self.ASSIST_PEAK_PHASE - self.PHASE_LEAD    # 中心相位，实际的助力峰值位置
        self.WINDOW_SIZE = 10
        # 另有alpha平滑参数在后面定义，用于滤波输出，越大越跟随实时变化

        # === 安全阈值 ===
        # 静止检测阈值 (你原本的参数)
        self.STATIC_VAR_THRES = 50.0  
        # 【新增】相位硬锁：低于此相位强制 0 力矩，防止支撑相误触
        self.PHASE_GATE_THRES = 0.45  

        # 3. 加载模型
        try:
            self.model = joblib.load(os.path.join(data_dir, 'optimized_gait_svm_model.pkl'))
            scaler_dict = joblib.load(os.path.join(data_dir, 'optimized_gait_scaler.pkl'))
            if isinstance(scaler_dict, dict):
                self.minmax = scaler_dict['minmax']
                self.std = scaler_dict['std']
            else:
                self.std = scaler_dict
                self.minmax = None
            self.get_logger().info('✅ 模型加载成功')
        except Exception as e:
            self.get_logger().error(f'❌ 模型加载失败: {e}')
            return

        # 4. 缓冲区与滤波
        self.history = deque(maxlen=self.WINDOW_SIZE)
        self.last_torque = 0.0 # 用于平滑滤波

        # 5. 通信
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pub_torque = self.create_publisher(Float32, '/control/cmd_torque', 10)

    def imu_callback(self, msg):
        # --- 数据预处理 (保持不变) ---
        roll_rad, pitch_rad, yaw_rad = self.quat_to_euler(msg.orientation)
        deg_factor = 180.0 / math.pi
        roll, pitch, yaw = roll_rad*deg_factor, pitch_rad*deg_factor, yaw_rad*deg_factor
        
        acc_factor = 1000.0 / 9.80665
        acc_x = msg.linear_acceleration.x * acc_factor
        acc_y = msg.linear_acceleration.y * acc_factor
        acc_z = msg.linear_acceleration.z * acc_factor
        
        gyro_factor = 180.0 / math.pi
        gyro_x = msg.angular_velocity.x * gyro_factor
        gyro_y = msg.angular_velocity.y * gyro_factor
        gyro_z = msg.angular_velocity.z * gyro_factor

        row = [roll, pitch, yaw, acc_x, acc_y, gyro_x, gyro_y, gyro_z, acc_z]
        self.history.append(row)

        if len(self.history) == self.WINDOW_SIZE:
            # === 安全锁 1: 静止检测 ===
            history_np = np.array(self.history)
            acc_z_std = np.std(history_np[:, 8])
            gyro_x_std = np.std(history_np[:, 5])
            
            IS_STATIC = (acc_z_std < self.STATIC_VAR_THRES) and (gyro_x_std < 10.0)
            
            target_tau = 0.0
            phase = 0.0

            if IS_STATIC:
                # 静止状态：强制归零
                target_tau = 0.0
                if self.count % 100 == 0:
                    self.get_logger().info(f'🛑 [STATIC] StdZ:{acc_z_std:.1f} (Threshold: {self.STATIC_VAR_THRES})')
            else:
                # 运动状态：SVM 预测
                feats = self.compute_features(self.history)
                if self.minmax: feats = self.minmax.transform(feats)
                feats = self.std.transform(feats)
                
                phase = self.model.predict(feats)[0]
                phase = float(np.clip(phase, 0.0, 1.0))
                
                # === 安全锁 2: 相位截断 (Critical!) ===
                # 无论 SVM 预测什么，只要小于 0.45 (支撑相)，物理层禁止助力
                if phase < self.PHASE_GATE_THRES:
                    target_tau = 0.0
                else:
                    target_tau = self.gaussian_assist(phase)

            # === 平滑滤波 (防止力矩突变) ===
            # alpha 越小越平滑，延迟越大。0.3 是个折中值
            alpha = 0.8
            smooth_tau = alpha * target_tau + (1 - alpha) * self.last_torque
            self.last_torque = smooth_tau
            
            # 发布指令
            out_msg = Float32()
            out_msg.data = float(smooth_tau)
            self.pub_torque.publish(out_msg)

            # 日志
            self.count += 1
            if smooth_tau > 0.1:
                if self.count % 10 == 0:
                     self.get_logger().info(f'🔥 [BOOST] Ph:{phase:.2f} | Tau:{smooth_tau:.2f} Nm')

    def gaussian_assist(self, phase):
        # 环形距离计算 (这不是阈值！)
        dist = abs(phase - self.CENTER_PHASE)
        if dist > 0.5: dist = 1.0 - dist
        
        # 计算高斯力矩
        torque = self.MAX_TORQUE * np.exp(-(dist**2) / (2 * self.ASSIST_WIDTH**2))
        return torque if torque > 0.1 else 0.0

    def compute_features(self, buffer):
        data = np.array(buffer).T
        feats = []
        for ch in data:
            feats.append(detrend(ch)[-1])
        for ch in data:
            feats.append(np.mean(ch))
            feats.append(np.std(ch))
            feats.append(np.max(ch))
        return np.array(feats).reshape(1, -1)
        
    def quat_to_euler(self, q):
        # ... (保持原样) ...
        w, x, y, z = q.w, q.x, q.y, q.z
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = GaitInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()