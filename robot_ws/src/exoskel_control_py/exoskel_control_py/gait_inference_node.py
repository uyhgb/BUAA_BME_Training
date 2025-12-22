#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
步态推理节点
功能: 订阅IMU数据 -> 特征提取 -> SVM预测步态相位 -> 计算助力力矩
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
        
        # 1. 参数配置
        pkg_dir = get_package_share_directory('exoskel_control_py')
        self.declare_parameter('model_path', os.path.join(pkg_dir, 'config'))
        data_dir = self.get_parameter('model_path').value
        
        # 助力策略参数 (60% 处助力)
        self.ASSIST_PEAK_PHASE = 0.60
        self.PHASE_LEAD = 0.05
        self.ASSIST_WIDTH = 0.10
        self.MAX_TORQUE = 3.0  # 安全起见先设小
        self.CENTER_PHASE = self.ASSIST_PEAK_PHASE - self.PHASE_LEAD
        self.WINDOW_SIZE = 10

        # 2. 加载模型
        try:
            self.model = joblib.load(os.path.join(data_dir, 'optimized_gait_svm_model.pkl'))
            scaler_dict = joblib.load(os.path.join(data_dir, 'optimized_gait_scaler.pkl'))
            # 处理 scaler 格式
            if isinstance(scaler_dict, dict):
                self.minmax = scaler_dict['minmax']
                self.std = scaler_dict['std']
            else:
                self.std = scaler_dict
                self.minmax = None
            self.get_logger().info('✅ SVM 模型加载成功')
        except Exception as e:
            self.get_logger().error(f'❌ 模型加载失败: {e}')
            raise e

        # 3. 数据缓冲区
        self.history = deque(maxlen=self.WINDOW_SIZE)

        # 4. 通信接口
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.pub_torque = self.create_publisher(Float32, '/control/cmd_torque', 10)

    def imu_callback(self, msg):
        """处理 IMU 数据 -> 预测 -> 发布力矩"""
        # 提取 9轴数据 (顺序必须与训练一致: Roll, Pitch, Yaw, AccX, AccY, GyroX... AccZ)
        # 注意：这里需要你根据实际 IMU 坐标系转换 quaternion 到 euler
        # 简单起见，这里假设 msg.orientation 已经被 Cpp 节点转成了 euler 放在某些字段，
        # 或者在这里做四元数转欧拉角。为简化代码，这里假设 C++ 发的就是处理好的数据或者我们在内部转。
        
        # 临时：假设 msg.angular_velocity 和 linear_acceleration 是原始数据
        # 真实的四元数转欧拉角代码略，假设 roll, pitch, yaw 已计算
        roll, pitch, yaw = self.quat_to_euler(msg.orientation)
        
        row = [
            roll, pitch, yaw,
            msg.linear_acceleration.x, msg.linear_acceleration.y,
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.z
        ]
        
        self.history.append(row)

        if len(self.history) == self.WINDOW_SIZE:
            # 特征工程
            features = self.compute_features(self.history)
            
            # 标准化
            if self.minmax:
                features = self.minmax.transform(features)
            features = self.std.transform(features)
            
            # 预测相位
            phase = self.model.predict(features)[0]
            
            # 计算稀疏助力 (高斯曲线)
            target_tau = self.gaussian_assist(phase)
            
            # 发布命令
            out_msg = Float32()
            out_msg.data = float(target_tau)
            self.pub_torque.publish(out_msg)
            
            if target_tau > 0.1:
                self.get_logger().info(f'Phase: {phase:.2f} | Torque: {target_tau:.2f} Nm')

    def gaussian_assist(self, phase):
        dist = abs(phase - self.CENTER_PHASE)
        if dist > 0.5: dist = 1.0 - dist
        torque = self.MAX_TORQUE * np.exp(-(dist**2) / (2 * self.ASSIST_WIDTH**2))
        return torque if torque > 0.1 else 0.0

    def compute_features(self, buffer):
        data = np.array(buffer).T
        feats = []
        # Detrend last
        for ch in data:
            feats.append(detrend(ch)[-1])
        # Stats
        for ch in data:
            feats.append(np.mean(ch))
            feats.append(np.std(ch))
            feats.append(np.max(ch))
        return np.array(feats).reshape(1, -1)
        
    def quat_to_euler(self, q):
        """四元数转欧拉角 (Roll, Pitch, Yaw)"""
        # q.w, q.x, q.y, q.z
        w, x, y, z = q.w, q.x, q.y, q.z
        
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
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