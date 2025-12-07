#!/usr/bin/env python3
"""
IMU 健康检查工具
用于检测 IMU 数据的基本健康状况
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import time


class IMUHealthChecker(Node):
    def __init__(self):
        super().__init__('imu_health_checker')
        
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # 数据缓冲
        self.acc_samples = []
        self.gyro_samples = []
        self.orientation_samples = []
        
        # 统计参数
        self.sample_count = 0
        self.max_samples = 100  # 收集 100 个样本
        self.start_time = time.time()
        
        self.get_logger().info('='*60)
        self.get_logger().info('IMU 健康检查工具已启动')
        self.get_logger().info('请确保 IMU 静止放置在水平桌面上...')
        self.get_logger().info(f'将收集 {self.max_samples} 个样本进行分析...')
        self.get_logger().info('='*60)
    
    def imu_callback(self, msg):
        if self.sample_count >= self.max_samples:
            return
        
        # 收集加速度数据
        acc = [msg.linear_acceleration.x, 
               msg.linear_acceleration.y, 
               msg.linear_acceleration.z]
        self.acc_samples.append(acc)
        
        # 收集角速度数据
        gyro = [msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z]
        self.gyro_samples.append(gyro)
        
        # 收集姿态数据
        quat = [msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w]
        self.orientation_samples.append(quat)
        
        self.sample_count += 1
        
        # 每 20 个样本显示一次进度
        if self.sample_count % 20 == 0:
            self.get_logger().info(f'已收集 {self.sample_count}/{self.max_samples} 个样本...')
        
        # 收集完成后进行分析
        if self.sample_count == self.max_samples:
            self.analyze_data()
    
    def analyze_data(self):
        elapsed_time = time.time() - self.start_time
        
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('数据收集完成,开始分析...')
        self.get_logger().info('='*60)
        
        # 转换为 numpy 数组
        acc_data = np.array(self.acc_samples)
        gyro_data = np.array(self.gyro_samples)
        
        # ==================== 采样率检查 ====================
        sample_rate = self.max_samples / elapsed_time
        self.get_logger().info('')
        self.get_logger().info('【采样率】')
        self.get_logger().info(f'  实际采样率: {sample_rate:.2f} Hz')
        if sample_rate < 5:
            self.get_logger().warn(f'  ⚠️  采样率过低!')
        elif sample_rate < 15:
            self.get_logger().info(f'  ⚠️  采样率偏低 (期望 20 Hz)')
        else:
            self.get_logger().info(f'  ✓ 采样率正常')
        
        # ==================== 加速度分析 ====================
        self.get_logger().info('')
        self.get_logger().info('【加速度分析】(单位: m/s²)')
        
        acc_mean = np.mean(acc_data, axis=0)
        acc_std = np.std(acc_data, axis=0)
        acc_magnitude = np.linalg.norm(acc_mean)
        
        self.get_logger().info(f'  均值: X={acc_mean[0]:7.2f}, Y={acc_mean[1]:7.2f}, Z={acc_mean[2]:7.2f}')
        self.get_logger().info(f'  标准差: X={acc_std[0]:7.2f}, Y={acc_std[1]:7.2f}, Z={acc_std[2]:7.2f}')
        self.get_logger().info(f'  合加速度: {acc_magnitude:.2f} m/s² (理想值: 9.81 m/s²)')
        
        # 检查 Z 轴是否接近重力加速度
        gravity_error = abs(acc_mean[2] - 9.81)
        if gravity_error > 2.0:
            self.get_logger().warn(f'  ⚠️  Z轴加速度异常! 误差: {gravity_error:.2f} m/s²')
            self.get_logger().warn(f'      请检查: 1) IMU是否水平放置  2) 加速度单位是否为 m/s²')
        else:
            self.get_logger().info(f'  ✓ Z轴重力加速度正常')
        
        # 检查 X/Y 轴噪声
        if acc_std[0] > 50 or acc_std[1] > 50:
            self.get_logger().warn(f'  ⚠️  X/Y轴噪声过大!')
            self.get_logger().warn(f'      可能原因: 1) 桌面震动  2) IMU未固定  3) 传感器故障')
        elif acc_std[0] > 20 or acc_std[1] > 20:
            self.get_logger().info(f'  ⚠️  X/Y轴有一定噪声,建议检查固定情况')
        else:
            self.get_logger().info(f'  ✓ X/Y轴噪声在正常范围')
        
        # ==================== 角速度分析 ====================
        self.get_logger().info('')
        self.get_logger().info('【角速度分析】(单位: rad/s)')
        
        gyro_mean = np.mean(gyro_data, axis=0)
        gyro_std = np.std(gyro_data, axis=0)
        gyro_max = np.max(np.abs(gyro_data), axis=0)
        
        self.get_logger().info(f'  均值: X={gyro_mean[0]:7.4f}, Y={gyro_mean[1]:7.4f}, Z={gyro_mean[2]:7.4f}')
        self.get_logger().info(f'  标准差: X={gyro_std[0]:7.4f}, Y={gyro_std[1]:7.4f}, Z={gyro_std[2]:7.4f}')
        self.get_logger().info(f'  最大值: X={gyro_max[0]:7.4f}, Y={gyro_max[1]:7.4f}, Z={gyro_max[2]:7.4f}')
        
        # 检查零偏
        gyro_bias = np.abs(gyro_mean)
        if np.any(gyro_bias > 0.1):
            self.get_logger().warn(f'  ⚠️  陀螺仪零偏较大!')
            self.get_logger().warn(f'      建议进行陀螺仪校准')
        elif np.any(gyro_bias > 0.05):
            self.get_logger().info(f'  ⚠️  陀螺仪有轻微零偏,可接受')
        else:
            self.get_logger().info(f'  ✓ 陀螺仪零偏正常')
        
        # 检查变化率
        if np.any(gyro_std > 1.0):
            self.get_logger().warn(f'  ⚠️  角速度变化过大!')
            self.get_logger().warn(f'      请确认 IMU 完全静止')
        elif np.any(gyro_std > 0.5):
            self.get_logger().info(f'  ⚠️  角速度有波动,建议减少震动')
        else:
            self.get_logger().info(f'  ✓ 角速度稳定性良好')
        
        # ==================== 姿态分析 ====================
        self.get_logger().info('')
        self.get_logger().info('【姿态分析】')
        
        # 从四元数计算欧拉角
        roll_list = []
        pitch_list = []
        yaw_list = []
        
        for quat in self.orientation_samples:
            x, y, z, w = quat
            # Roll (X轴旋转)
            roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
            # Pitch (Y轴旋转)
            pitch = np.arcsin(2*(w*y - z*x))
            # Yaw (Z轴旋转)
            yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            
            roll_list.append(np.degrees(roll))
            pitch_list.append(np.degrees(pitch))
            yaw_list.append(np.degrees(yaw))
        
        roll_mean = np.mean(roll_list)
        pitch_mean = np.mean(pitch_list)
        yaw_mean = np.mean(yaw_list)
        roll_std = np.std(roll_list)
        pitch_std = np.std(pitch_list)
        yaw_std = np.std(yaw_list)
        
        self.get_logger().info(f'  Roll:  均值={roll_mean:7.2f}°, 标准差={roll_std:.3f}°')
        self.get_logger().info(f'  Pitch: 均值={pitch_mean:7.2f}°, 标准差={pitch_std:.3f}°')
        self.get_logger().info(f'  Yaw:   均值={yaw_mean:7.2f}°, 标准差={yaw_std:.3f}°')
        
        # 检查水平度
        if abs(roll_mean) > 10 or abs(pitch_mean) > 10:
            self.get_logger().warn(f'  ⚠️  IMU 未水平放置!')
            self.get_logger().warn(f'      请调整 IMU 使其尽量水平')
        elif abs(roll_mean) > 5 or abs(pitch_mean) > 5:
            self.get_logger().info(f'  ⚠️  IMU 倾斜 {max(abs(roll_mean), abs(pitch_mean)):.1f}°')
        else:
            self.get_logger().info(f'  ✓ IMU 放置水平')
        
        # 检查姿态稳定性
        if roll_std > 2 or pitch_std > 2 or yaw_std > 2:
            self.get_logger().warn(f'  ⚠️  姿态抖动较大!')
            self.get_logger().warn(f'      可能原因: 传感器噪声或融合算法不稳定')
        elif roll_std > 0.5 or pitch_std > 0.5 or yaw_std > 0.5:
            self.get_logger().info(f'  ⚠️  姿态有轻微抖动')
        else:
            self.get_logger().info(f'  ✓ 姿态稳定')
        
        # ==================== 综合评估 ====================
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('【综合评估】')
        
        issues = []
        warnings = []
        
        if sample_rate < 5:
            issues.append('采样率过低')
        elif sample_rate < 15:
            warnings.append('采样率偏低')
        
        if gravity_error > 2.0:
            issues.append('重力加速度异常')
        
        if acc_std[0] > 50 or acc_std[1] > 50:
            issues.append('加速度噪声过大')
        elif acc_std[0] > 20 or acc_std[1] > 20:
            warnings.append('加速度有噪声')
        
        if np.any(gyro_bias > 0.1):
            warnings.append('陀螺仪零偏较大')
        
        if np.any(gyro_std > 1.0):
            issues.append('角速度不稳定')
        elif np.any(gyro_std > 0.5):
            warnings.append('角速度有波动')
        
        if abs(roll_mean) > 10 or abs(pitch_mean) > 10:
            warnings.append('IMU 未水平放置')
        
        if roll_std > 2 or pitch_std > 2 or yaw_std > 2:
            warnings.append('姿态抖动较大')
        
        if len(issues) == 0 and len(warnings) == 0:
            self.get_logger().info('  ✓✓✓ IMU 工作正常! ✓✓✓')
        else:
            if issues:
                self.get_logger().error(f'  发现 {len(issues)} 个严重问题:')
                for issue in issues:
                    self.get_logger().error(f'    - {issue}')
            
            if warnings:
                self.get_logger().warn(f'  发现 {len(warnings)} 个警告:')
                for warning in warnings:
                    self.get_logger().warn(f'    - {warning}')
        
        self.get_logger().info('='*60)
        self.get_logger().info('检查完成,节点将在 3 秒后退出...')
        
        # 延迟退出
        time.sleep(3)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    checker = IMUHealthChecker()
    
    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        pass
    finally:
        checker.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
