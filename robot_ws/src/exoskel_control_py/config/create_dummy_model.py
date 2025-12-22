#!/usr/bin/env python3
"""
临时脚本：生成虚拟 SVM 模型文件以便系统能启动
注意：这不是真实训练的模型，只是为了让节点不崩溃
"""
import joblib
import numpy as np
from sklearn.svm import SVC
from sklearn.preprocessing import StandardScaler

# 创建一个虚拟的 SVM 模型（未经训练，随机初始化）
print("正在创建虚拟 SVM 模型...")
dummy_model = SVC(kernel='rbf', C=1.0, gamma='scale')

# 用少量虚拟数据训练（让模型有基本结构）
X_dummy = np.random.randn(100, 8)  # 假设 8 个特征
y_dummy = np.random.randint(0, 2, 100)  # 二分类 (swing/stance)
dummy_model.fit(X_dummy, y_dummy)

# 创建虚拟的 scaler
dummy_scaler = StandardScaler()
dummy_scaler.fit(X_dummy)

# 保存模型
joblib.dump(dummy_model, 'optimized_gait_svm_model.pkl')
joblib.dump(dummy_scaler, 'optimized_gait_scaler.pkl')

print("✅ 虚拟模型已创建:")
print("  - optimized_gait_svm_model.pkl")
print("  - optimized_gait_scaler.pkl")
print("\n⚠️  警告：这是未训练的虚拟模型，步态推理结果不准确")
print("   请在后续训练真实模型后替换这些文件")
