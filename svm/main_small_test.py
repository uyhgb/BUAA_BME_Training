# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
import joblib
import os
import warnings
import sys
from scipy.signal import detrend, find_peaks
from sklearn.preprocessing import StandardScaler, MinMaxScaler
from sklearn.svm import SVR
from sklearn.model_selection import train_test_split, GridSearchCV
from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
from sklearn.utils.class_weight import compute_sample_weight
from sklearn.ensemble import RandomForestRegressor

# 关闭不必要的警告
warnings.filterwarnings('ignore')

# ===================== 1. 核心配置 (修改为你的小文件) =====================
BASE_DIR = "./"
# 这里指向你刚才清洗好的那个 1万行的小文件 (Part 2)
RAW_IMU_PATH = os.path.join(BASE_DIR, "imu_gait_data_20251220_213756.csv") 

# 输出文件命名带上 _test 后缀，避免覆盖正式模型
LABELED_IMU_PATH = os.path.join(BASE_DIR, "labeled_data_test.csv")
MODEL_OUTPUT_PATH = os.path.join(BASE_DIR, "svm_model_test.pkl")
SCALER_OUTPUT_PATH = os.path.join(BASE_DIR, "scaler_test.pkl")
FEATURE_COLS_PATH = os.path.join(BASE_DIR, "features_test.pkl")

# ===================== 2. 辅助函数 =====================
def get_gait_phase(pos):
    """将0-1的步态周期位置划分为5个阶段"""
    pos = np.clip(pos, 0, 1)
    if 0.0 <= pos < 0.2: return 0
    elif 0.2 <= pos < 0.4: return 1
    elif 0.4 <= pos < 0.6: return 2
    elif 0.6 <= pos < 0.8: return 3
    else: return 4

def calculate_metrics(y_true, y_pred):
    r2 = r2_score(y_true, y_pred)
    rmse = np.sqrt(mean_squared_error(y_true, y_pred))
    
    # 步态阶段分类准确率
    y_true_phase = np.array([get_gait_phase(p) for p in y_true])
    y_pred_phase = np.array([get_gait_phase(p) for p in y_pred])
    acc = (y_true_phase == y_pred_phase).sum() / len(y_true_phase) * 100
    
    return {"R²": r2, "RMSE": rmse, "Phase_Acc": acc}

# ===================== 3. 标签生成 (核心算法) =====================
def generate_gait_labels(df_raw):
    print(f"[2/5] 生成步态标签 (峰值检测 + 插值)...")
    df = df_raw.copy()
    
    # 简单的去趋势处理
    df['AccZ_detrend'] = detrend(df['AccZ'].values)
    
    # 峰值检测 (distance=60 对应 0.6s 最小间隔)
    peaks, _ = find_peaks(
        df['AccZ_detrend'],
        height=np.percentile(df['AccZ_detrend'], 60), 
        distance=60,  
        prominence=0.5
    )
    
    if len(peaks) < 2:
        raise ValueError("峰值太少，无法生成标签")

    print(f"   检测到 {len(peaks)} 个步态峰值")

    # 逐周期插值
    df['GaitCycleLabel'] = -1.0
    for i in range(len(peaks) - 1):
        start, end = peaks[i], peaks[i+1]
        length = end - start
        df.iloc[start:end, df.columns.get_loc('GaitCycleLabel')] = np.linspace(0, 1, length, endpoint=False)
        
    # 只保留有效数据
    df_valid = df[df['GaitCycleLabel'] != -1.0].copy()
    df_valid['GaitPhase'] = df_valid['GaitCycleLabel'].apply(get_gait_phase)
    
    return df_valid

# ===================== 4. 特征工程 =====================
def preprocess_features(df):
    print(f"[3/5] 提取特征 (滑动窗口)...")
    
    # 基础去趋势
    cols = ['Roll', 'Pitch', 'Yaw', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ']
    for c in cols:
        df[f'{c}_detrend'] = detrend(df[c].values)
    
    # 时域特征 (Window=10)
    feature_cols = []
    for c in cols:
        base_col = f'{c}_detrend'
        df[f'{c}_mean'] = df[base_col].rolling(10, center=True, min_periods=1).mean()
        df[f'{c}_std'] = df[base_col].rolling(10, center=True, min_periods=1).std()
        feature_cols.extend([base_col, f'{c}_mean', f'{c}_std'])
        
    df[feature_cols] = df[feature_cols].fillna(method='bfill').fillna(method='ffill')
    joblib.dump(feature_cols, FEATURE_COLS_PATH)
    
    return df[feature_cols], df['GaitCycleLabel'], df['GaitPhase']

# ===================== 主流程 =====================
if __name__ == "__main__":
    # A. 加载数据 (只读一个小文件)
    print(f"[1/5] 加载数据: {RAW_IMU_PATH}")
    df_raw = pd.read_csv(RAW_IMU_PATH)
    
    # 【极速测试关键点】：只取前 3000 行 (约30秒数据)
    # 这样既保证了连续性，又让训练秒级完成
    print(f"   原始数据量: {len(df_raw)}")
    df_small = df_raw.iloc[:3000].copy() 
    print(f"   >>> 截取前 3000 行进行极速验证 <<<")
    
    # B. 打标签
    df_labeled = generate_gait_labels(df_small)
    print(f"   有效训练样本: {len(df_labeled)}")
    
    # C. 特征工程 & 标准化
    X, y, y_phase = preprocess_features(df_labeled)
    
    print(f"[4/5] 数据标准化...")
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)
    joblib.dump(scaler, SCALER_OUTPUT_PATH)
    
    # D. 训练 SVM (参数写死，只跑一次)
    print(f"[5/5] 开始 SVM 极速训练...")
    
    # 固定参数，不搜索 (为了快)
    # C=100, gamma=0.1 是经验上比较好的值
    svr = SVR(kernel='rbf', C=100, gamma=0.1, tol=1e-3, verbose=True)
    
    # 划分数据
    X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, shuffle=True, random_state=42)
    
    # 训练
    svr.fit(X_train, y_train)
    
    # 评估
    y_pred = svr.predict(X_test)
    metrics = calculate_metrics(y_test, y_pred)
    
    print("\n" + "="*30)
    print(f"测试完成！极速验证结果：")
    print(f"R² (决定系数): {metrics['R²']:.4f}")
    print(f"RMSE (误差): {metrics['RMSE']:.4f}")
    print(f"分类准确率: {metrics['Phase_Acc']:.2f}%")
    print("="*30)
    
    # 保存小模型
    joblib.dump(svr, MODEL_OUTPUT_PATH)
    print(f"模型已保存: {MODEL_OUTPUT_PATH}")