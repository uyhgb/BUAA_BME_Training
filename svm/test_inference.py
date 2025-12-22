# -*- coding: utf-8 -*-
import joblib
import numpy as np
import pandas as pd
import time
import math
from collections import deque
from scipy.signal import detrend

# ================= 配置 =================
MODEL_PATH = 'optimized_gait_svm_model.pkl'
SCALER_PATH = 'optimized_gait_scaler.pkl'
COLS_PATH = 'optimized_feature_columns.pkl'
# 【关键】直接读取你的 CSV 文件进行回放测试
CSV_PATH = 'imu_gait_data_20251220_212156.csv' 

WINDOW_SIZE = 10 
SENSOR_ORDER = ['Roll', 'Pitch', 'Yaw', 'AccX', 'AccY', 'GyroX', 'GyroY', 'GyroZ', 'AccZ']

# ================= 工具函数 =================
def load_system():
    try:
        model = joblib.load(MODEL_PATH)
        scaler_dict = joblib.load(SCALER_PATH)
        cols = joblib.load(COLS_PATH)
        
        # 提取 Scaler
        if isinstance(scaler_dict, dict):
            minmax = scaler_dict['minmax']
            std = scaler_dict['std']
        else:
            minmax = None
            std = scaler_dict
            
        print(f"✅ 模型加载完毕 (特征数: {len(cols)})")
        return model, minmax, std, cols
    except Exception as e:
        print(f"❌ 加载失败: {e}")
        exit()

def compute_features(buffer):
    # 转置: (9, 10)
    data = np.array(buffer).T
    
    # 1. 基础特征 (9个): 去趋势后的最新值
    base_features = []
    for channel_data in data:
        detrended = detrend(channel_data)
        base_features.append(detrended[-1])
        
    # 2. 统计特征 (27个): Mean, Std, Max
    stats_features = []
    for channel_data in data:
        stats_features.append(np.mean(channel_data))
        stats_features.append(np.std(channel_data))
        stats_features.append(np.max(channel_data))
        
    return np.array(base_features + stats_features).reshape(1, -1)

# ================= 主程序 =================
def main():
    model, minmax, std, cols = load_system()
    
    # 1. 读取真实数据 (取前 500 行作为测试集)
    print(f">> 正在读取 CSV 文件: {CSV_PATH} ...")
    try:
        df_real = pd.read_csv(CSV_PATH)
        # 确保列顺序一致
        df_replay = df_real[SENSOR_ORDER].iloc[:500].values
        print(f"✅ 成功加载 {len(df_replay)} 行真实数据用于回放测试")
    except Exception as e:
        print(f"❌ 读取 CSV 失败: {e}")
        exit()

    history = deque(maxlen=WINDOW_SIZE)
    
    print("\n>> 开始回放真实数据推理...")
    print(f"{'Row':<6} | {'AccZ(Raw)':<10} | {'Phase(%)':<10} | {'Torque':<10}")
    print("-" * 50)
    
    # 2. 逐行回放
    for i, row in enumerate(df_replay):
        # row 就是真实的 [Roll, Pitch, ... AccZ]
        history.append(row)
        
        if len(history) == WINDOW_SIZE:
            # A. 特征计算
            raw_feats = compute_features(history)
            
            # B. 双重标准化
            feats_mm = minmax.transform(raw_feats)
            feats_ready = std.transform(feats_mm)
            
            # C. 预测
            phase = model.predict(feats_ready)[0]
            
            # D. 计算力矩 (演示用)
            tau = 0.0
            if 0 < phase < 1:
                tau = 23.0 * math.sin(phase * 3.14)
            
            # 打印
            if i % 10 == 0: # 每10行打印一次
                 print(f"{i:<6} | {row[8]:<10.1f} | {phase*100:<6.1f}%    | {tau:<6.2f}")
        
        time.sleep(0.01) # 模拟 100Hz

if __name__ == "__main__":
    main()