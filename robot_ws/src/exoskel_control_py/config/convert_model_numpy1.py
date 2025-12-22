#!/usr/bin/env python3
"""
将 NumPy 2.x 保存的模型转换为 NumPy 1.x 兼容格式
"""
import sys
import os

# 临时升级到 NumPy 2.x 来加载
os.system('pip3 install "numpy>=2.0" --quiet')

import joblib
import numpy as np

print(f"当前 NumPy 版本: {np.__version__}")

# 加载原始模型（NumPy 2.x）
print("正在加载原始模型...")
model = joblib.load('optimized_gait_svm_model.pkl')
scaler = joblib.load('optimized_gait_scaler.pkl')
feature_cols = joblib.load('optimized_feature_columns.pkl')

print(f"✅ 模型加载成功")
print(f"   - SVM 类型: {type(model)}")
print(f"   - Scaler 类型: {type(scaler)}")
print(f"   - 特征列: {len(feature_cols)} 个")

# 降级到 NumPy 1.x
print("\n正在降级 NumPy 到 1.x...")
os.system('pip3 install "numpy==1.24.4" --quiet')

# 重新导入 joblib（使用新的 NumPy）
import importlib
importlib.reload(joblib)

# 重新保存（现在用 NumPy 1.x 序列化）
print(f"当前 NumPy 版本: {np.__version__}")
print("正在用 NumPy 1.x 重新保存模型...")
joblib.dump(model, 'optimized_gait_svm_model.pkl')
joblib.dump(scaler, 'optimized_gait_scaler.pkl')
joblib.dump(feature_cols, 'optimized_feature_columns.pkl')

print("\n✅ 模型转换完成，现在兼容 NumPy 1.24.x")
print("请重新编译 ROS2 包: colcon build --packages-select exoskel_control_py")
