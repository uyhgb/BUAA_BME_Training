# -*- coding: utf-8 -*-
# 尝试合并两个文件
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
from sklearn.ensemble import RandomForestRegressor  # 备用模型


# 关闭收敛警告（仅简化输出，核心问题已通过参数优化解决）
warnings.filterwarnings('ignore', category=UserWarning)
warnings.filterwarnings('ignore', category=FutureWarning)
warnings.filterwarnings('ignore', category=DeprecationWarning)

# ===================== 核心配置 =====================
BASE_DIR = "./"
RAW_IMU_PATH = os.path.join(BASE_DIR, "imu_gait_data_20251220_212156.csv")
LABELED_IMU_PATH = os.path.join(BASE_DIR, "labeled_imu_data_combined.csv")
MODEL_OUTPUT_PATH = os.path.join(BASE_DIR, "optimized_gait_svm_model.pkl")
SCALER_OUTPUT_PATH = os.path.join(BASE_DIR, "optimized_gait_scaler.pkl")
FEATURE_COLS_PATH = os.path.join(BASE_DIR, "optimized_feature_columns.pkl")

# ===================== 步态阶段划分 + 数据平衡函数 =====================
def get_gait_phase(pos):
    """将0-1的步态周期位置划分为5个阶段"""
    pos = np.clip(pos, 0, 1)
    if 0.0 <= pos < 0.2:
        return 0  # 支撑相早期
    elif 0.2 <= pos < 0.4:
        return 1  # 支撑相中期
    elif 0.4 <= pos < 0.6:
        return 2  # 支撑相晚期
    elif 0.6 <= pos < 0.8:
        return 3  # 摆动相早期
    else:
        return 4  # 摆动相晚期

def balance_gait_phase_data(df):
    """平衡各步态阶段样本数（过采样少数类，欠采样多数类）"""
    print("平衡步态阶段样本分布...")
    # 按阶段分组
    phase_groups = [df[df['GaitPhase'] == i] for i in range(5)]
    # 计算目标样本数（取中位数）
    target_samples = int(np.median([len(g) for g in phase_groups]))
    
    balanced_dfs = []
    for i, group in enumerate(phase_groups):
        if len(group) == 0:
            continue
        if len(group) > target_samples:
            # 欠采样多数类
            balanced_group = group.sample(n=target_samples, random_state=42)
        else:
            # 过采样少数类
            balanced_group = group.sample(n=target_samples, replace=True, random_state=42)
        balanced_dfs.append(balanced_group)
    
    balanced_df = pd.concat(balanced_dfs, ignore_index=True)
    print(f"平衡前样本分布：{df['GaitPhase'].value_counts().sort_index().to_dict()}")
    print(f"平衡后样本分布：{balanced_df['GaitPhase'].value_counts().sort_index().to_dict()}")
    return balanced_df

# ===================== 评估指标计算 =====================
def calculate_regression_metrics(y_true, y_pred):
    """计算回归任务核心指标"""
    r2 = r2_score(y_true, y_pred)
    mae = mean_absolute_error(y_true, y_pred)
    rmse = np.sqrt(mean_squared_error(y_true, y_pred))
    # 修复MAPE计算（避免除以0）
    y_true_nonzero = np.where(y_true == 0, 1e-8, y_true)
    mape = np.mean(np.abs((y_true - y_pred) / y_true_nonzero)) * 100
    
    # 步态阶段分类准确率
    y_true_phase = np.array([get_gait_phase(p) for p in y_true])
    y_pred_phase = np.array([get_gait_phase(p) for p in y_pred])
    phase_accuracy = (y_true_phase == y_pred_phase).sum() / len(y_true_phase) * 100
    
    return {
        "R²决定系数": r2,
        "MAE平均绝对误差": mae,
        "RMSE均方根误差": rmse,
        "MAPE平均绝对百分比误差(%)": mape,
        "步态阶段分类准确率(%)": phase_accuracy
    }

# ===================== 数据生成/加载 =====================
def generate_raw_imu_data():
    print(f"[1/7] 生成模拟无标签IMU数据到：{RAW_IMU_PATH}")
    np.random.seed(42)
    n_samples = 2000
    timestamp = np.arange(n_samples) * 20
    
    gait_cycle = 1.2
    time_sec = timestamp / 1000
    acc_z_base = 980
    acc_z_gait = 15 * np.sin(2 * np.pi * time_sec / gait_cycle)
    acc_z_noise = np.random.normal(0, 1, n_samples)
    
    df = pd.DataFrame({
        'Timestamp': timestamp,
        'Roll': np.random.normal(-0.2, 0.1, n_samples),
        'Pitch': np.random.normal(-0.05, 0.02, n_samples),
        'Yaw': np.random.normal(158, 1, n_samples),
        'AccX': np.random.normal(2, 0.5, n_samples),
        'AccY': np.random.normal(-3, 0.8, n_samples),
        'AccZ': acc_z_base + acc_z_gait + acc_z_noise,
        'GyroX': np.random.normal(0.3, 0.1, n_samples),
        'GyroY': np.random.normal(0.2, 0.1, n_samples),
        'GyroZ': np.random.normal(0.1, 0.1, n_samples)
    })
    
    df.to_csv(RAW_IMU_PATH, index=False)
    print(f"模拟无标签数据生成完成，样本数：{len(df)}")
    return df

# ===================== 标签生成（优化峰值检测） =====================
def generate_gait_labels(df_raw):
    print(f"[2/7] 生成步态标签（基于逐周期插值）...")
    df = df_raw.copy()
    
    # 1. 基础信号处理
    df['RelativeTime'] = (df['Timestamp'] - df['Timestamp'].min()) / 1000
    
    # 去除AccZ的异常突刺
    acc_z_median = df['AccZ'].median()
    acc_z_std = df['AccZ'].std()
    df['AccZ_clean'] = np.where(
        np.abs(df['AccZ'] - acc_z_median) > 3 * acc_z_std,
        acc_z_median,
        df['AccZ']
    )
    df['AccZ_detrended'] = detrend(df['AccZ_clean'].values)
    
    # 2. 峰值检测 (针对100Hz优化)
    # distance=60: 意味着最小间隔0.6秒 (对于1s左右的步态很合适)
    peaks, _ = find_peaks(
        df['AccZ_detrended'],
        height=np.percentile(df['AccZ_detrended'], 60), 
        distance=60,  
        prominence=0.5
    )
    
    if len(peaks) < 2:
        raise ValueError(f"错误：仅检测到 {len(peaks)} 个步态峰值，无法进行训练。请检查数据质量。")

    print(f"成功检测到 {len(peaks)} 个步态周期，平均周期: {np.mean(np.diff(df['RelativeTime'].iloc[peaks])):.3f}s")

    # 3. 逐周期间插值 (解决相位漂移的关键步骤)
    # 初始化标签为 -1 (表示无效区域)
    df['GaitCycleLabel'] = -1.0
    
    peak_indices = peaks
    
    # 遍历每一对峰值，进行线性插值
    for i in range(len(peak_indices) - 1):
        start_idx = peak_indices[i]
        end_idx = peak_indices[i+1]
        
        # 这个区间的长度
        length = end_idx - start_idx
        
        # 生成 0.0 到 1.0 的线性序列
        phase_values = np.linspace(0, 1, length, endpoint=False)
        
        # 填入DataFrame
        df.iloc[start_idx:end_idx, df.columns.get_loc('GaitCycleLabel')] = phase_values
        
    # 处理最后一个峰值之后的数据（可选：丢弃或标记）
    # 这里我们只保留两个峰值之间的有效数据用于训练
    valid_mask = df['GaitCycleLabel'] != -1.0
    df_valid = df[valid_mask].copy()
    
    print(f"保留有效训练样本数: {len(df_valid)} (丢弃首尾无效数据)")

    # 4. 生成离散相位标签 (用于分类/平衡)
    df_valid['GaitPhase'] = df_valid['GaitCycleLabel'].apply(get_gait_phase)
    
    return df_valid

# ===================== 特征工程（新增时域特征） =====================
def preprocess_features(df_labeled):
    print(f"[3/7] 预处理特征（新增时域特征）...")
    df = df_labeled.copy()
    
    # 基础预处理
    acc_z_median = df['AccZ'].median()
    acc_z_std = df['AccZ'].std()
    df['AccZ_clean'] = np.where(
        np.abs(df['AccZ'] - acc_z_median) > 3 * acc_z_std,
        acc_z_median,
        df['AccZ']
    )
    
    # 去趋势
    trend_cols = ['Roll', 'Pitch', 'Yaw', 'AccX', 'AccY', 'GyroX', 'GyroY', 'GyroZ']
    for col in trend_cols:
        df[f'{col}_detrended'] = detrend(df[col].values)
    df['AccZ_detrended'] = detrend(df['AccZ_clean'].values)
    
    # 新增时域特征（提升模型区分度）
    window_size = 10  # 滑动窗口大小
    for col in trend_cols + ['AccZ_clean']:
        df[f'{col}_mean'] = df[col].rolling(window=window_size, center=True, min_periods=1).mean()
        df[f'{col}_std'] = df[col].rolling(window=window_size, center=True, min_periods=1).std()
        df[f'{col}_max'] = df[col].rolling(window=window_size, center=True, min_periods=1).max()
    
    # 特征列（基础特征 + 时域特征）
    base_features = [f'{col}_detrended' for col in trend_cols] + ['AccZ_detrended']
    time_features = []
    for col in trend_cols + ['AccZ_clean']:
        time_features.extend([f'{col}_mean', f'{col}_std', f'{col}_max'])
    feature_cols = base_features + time_features
    
    # 填充缺失值（滑动窗口导致）
    df[feature_cols] = df[feature_cols].fillna(method='bfill').fillna(method='ffill')
    
    joblib.dump(feature_cols, FEATURE_COLS_PATH)
    print(f"特征列已保存（共{len(feature_cols)}个特征）：{FEATURE_COLS_PATH}")
    
    return df[feature_cols], df['GaitCycleLabel'], df['GaitPhase']

# ===================== 数据标准化（双重标准化） =====================
def standardize_data(X):
    print(f"[4/7] 双重标准化特征数据...")
    # 第一步：MinMaxScaler缩放到[0,1]
    minmax_scaler = MinMaxScaler(feature_range=(0, 1))
    X_minmax = minmax_scaler.fit_transform(X.values)
    # 第二步：StandardScaler标准化
    std_scaler = StandardScaler()
    X_scaled = std_scaler.fit_transform(X_minmax)
    
    # 保存标准化器（组合保存）
    joblib.dump({'minmax': minmax_scaler, 'std': std_scaler}, SCALER_OUTPUT_PATH)
    print(f"标准化器已保存：{SCALER_OUTPUT_PATH}")
    return X_scaled, std_scaler

# ===================== 模型训练（优化SVM参数 + 样本权重） =====================
def train_optimized_model(X_scaled, y, y_phase):
    print(f"[5/7] 训练优化模型（7:3划分）...")
    # 7:3划分
    X_train, X_test, y_train, y_test, y_phase_train, y_phase_test = train_test_split(
        X_scaled, y, y_phase,
        test_size=0.3,
        random_state=42,
        shuffle=True,
        stratify=y_phase  # 分层抽样，保证阶段分布一致
    )
    
    print(f"数据集划分：")
    print(f"训练集样本数：{len(X_train)} ({len(X_train)/len(X_scaled)*100:.1f}%)")
    print(f"测试集样本数：{len(X_test)} ({len(X_test)/len(X_scaled)*100:.1f}%)")
    
    # 计算样本权重（解决类别失衡）
    sample_weights = compute_sample_weight('balanced', y_phase_train)
    
    # 优化SVM参数（解决收敛问题）
    param_grid = {
        'C': [50, 100, 200],  # 增大C值，提升拟合能力
        'gamma': [0.001, 0.01, 0.1],  # 缩小gamma范围
        'epsilon': [0.01, 0.05],
        'kernel': ['rbf'],
        'max_iter': [50000],  # 增大迭代次数
        'tol': [1e-4]  # 降低收敛阈值
    }
    
    print(f"开始网格搜索，这可能需要较长时间，请留意下方滚动日志...")

    # ================== 修改这里 ==================
    grid_search = GridSearchCV(
        estimator=SVR(verbose=False), # SVR本身保持安静，让GridSearch来汇报
        param_grid=param_grid,
        cv=3,              # 3折交叉验证
        scoring='r2',
        n_jobs=1,          # 单核运行（多核在Windows下可能不打印日志，单核最稳）
        
        # 【关键修改】：开启详细日志
        verbose=3,         
        
        error_score='raise'
    )
    # =============================================
    
    # 训练
    try:
        # 加一个时间戳打印，让你知道何时开始的
        import time
        start_time = time.time()
        print(f"[{time.strftime('%H:%M:%S')}] 开始拟合数据 (Fit)...")
        
        grid_search.fit(X_train, y_train, sample_weight=sample_weights)
        
        end_time = time.time()
        print(f"[{time.strftime('%H:%M:%S')}] 训练完成！耗时: {(end_time - start_time)/60:.1f} 分钟")
    except Exception as e:
        # 捕获异常并重新抛出（避免中文输出）
        error_msg = str(e).encode('ascii', errors='ignore').decode('ascii')
        raise RuntimeError(f"GridSearchCV failed: {error_msg}")
    
    best_model = grid_search.best_estimator_
    
    # 预测
    y_pred = best_model.predict(X_test)
    
    # 计算指标
    metrics = calculate_regression_metrics(y_test, y_pred)
    
    # 输出评估结果
    print(f"\n===== 模型评估结果 =====")
    for metric_name, value in metrics.items():
        if "%" in metric_name:
            print(f"{metric_name}：{value:.2f}")
        else:
            print(f"{metric_name}：{value:.4f}")
    
    # 各阶段准确率
    print(f"\n===== 各步态阶段分类准确率 =====")
    y_pred_phase = np.array([get_gait_phase(p) for p in y_pred])
    phase_names = ["支撑相早期", "支撑相中期", "支撑相晚期", "摆动相早期", "摆动相晚期"]
    for phase_id in range(5):
        phase_mask = (y_phase_test == phase_id)
        if phase_mask.sum() == 0:
            print(f"{phase_names[phase_id]}：无样本")
            continue
        phase_correct = (y_phase_test[phase_mask] == y_pred_phase[phase_mask]).sum()
        phase_acc = phase_correct / phase_mask.sum() * 100
        print(f"{phase_names[phase_id]}：{phase_acc:.2f}% (正确{phase_correct}/{phase_mask.sum()})")
    
    # 保存模型
    joblib.dump(best_model, MODEL_OUTPUT_PATH)
    print(f"\n最优模型已保存：{MODEL_OUTPUT_PATH}")
    print(f"最优参数：{grid_search.best_params_}")
    print(f"交叉验证R²：{grid_search.best_score_:.4f}")
    
    return best_model, metrics

# ===================== 备用模型（随机森林，防止SVM仍效果差） =====================
def train_fallback_model(X_scaled, y, y_phase):
    print(f"[5/7] SVM效果不佳，训练随机森林模型...")
    X_train, X_test, y_train, y_test, y_phase_train, y_phase_test = train_test_split(
        X_scaled, y, y_phase,
        test_size=0.3,
        random_state=42,
        stratify=y_phase
    )
    
    rf_model = RandomForestRegressor(
        n_estimators=100,
        max_depth=10,
        random_state=42,
        n_jobs=-1
    )
    rf_model.fit(X_train, y_train)
    
    y_pred = rf_model.predict(X_test)
    metrics = calculate_regression_metrics(y_test, y_pred)
    
    print(f"\n===== 随机森林模型评估 =====")
    for metric_name, value in metrics.items():
        if "%" in metric_name:
            print(f"{metric_name}：{value:.2f}")
        else:
            print(f"{metric_name}：{value:.4f}")
    
    joblib.dump(rf_model, MODEL_OUTPUT_PATH.replace("svm", "rf"))
    print(f"随机森林模型已保存：{MODEL_OUTPUT_PATH.replace('svm', 'rf')}")
    return rf_model, metrics

# ===================== 验证结果 =====================
def verify_results():
    print(f"[6/7] 验证生成结果...")
    files_to_check = [
        (LABELED_IMU_PATH, "带标签数据"),
        (MODEL_OUTPUT_PATH, "SVM模型"),
        (SCALER_OUTPUT_PATH, "标准化器"),
        (FEATURE_COLS_PATH, "特征列配置")
    ]
    
    all_ok = True
    for file_path, desc in files_to_check:
        if os.path.exists(file_path):
            size = os.path.getsize(file_path) / 1024
            print(f"{desc}：{file_path}（大小：{size:.2f} KB）")
        else:
            print(f"{desc}缺失：{file_path}")
            all_ok = False
    
    if all_ok:
        print("所有文件生成成功！")
    else:
        print("部分文件生成失败，请检查日志！")

# ===================== 主流程 =====================
if __name__ == "__main__":
    os.makedirs(BASE_DIR, exist_ok=True)
    
    # 1. 定义您的两个文件
    file_list = [
        "imu_gait_data_20251220_212156.csv",  # 3.2万行那个
        "imu_gait_data_20251220_213756.csv"   # 刚清洗好的1万行那个
    ]
    
    all_labeled_dfs = []

    print(f"=== 开始处理 {len(file_list)} 个数据文件 ===")
    
    # 2. 独立循环处理每个文件 (关键步骤！)
    for file_name in file_list:
        full_path = os.path.join(BASE_DIR, file_name)
        print(f"\n>> 正在加载: {file_name}")
        
        try:
            # 加载
            df_curr = pd.read_csv(full_path)
            print(f"   样本数: {len(df_curr)}")
            
            # 独立打标签 (去趋势、找峰值、插值)
            # 这一步必须分开做，否则两个文件的时间戳断层会破坏插值算法
            df_labeled = generate_gait_labels(df_curr)
            
            all_labeled_dfs.append(df_labeled)
            print(f"   成功生成标签，有效步态周期数: {int(df_labeled['GaitCycleLabel'].max()) if 'GaitCycleLabel' in df_labeled else 'N/A'}")
            
        except Exception as e:
            print(f"⚠️ 处理失败，跳过此文件: {e}")

    # 3. 合并数据
    if not all_labeled_dfs:
        print("没有有效数据，程序退出。")
        exit()
        
    print(f"\n>> 合并所有数据...")
    df_combined = pd.concat(all_labeled_dfs, axis=0, ignore_index=True)
    print(f"   总样本量: {len(df_combined)}")

    # ==========================================
    # 【新增】必须加上这一行，把合并后的数据存盘！
    # 这样上面的 LABELED_IMU_PATH 配置才会有意义
    # ==========================================
    print(f">> 保存合并后的带标签数据到: {LABELED_IMU_PATH}")
    df_combined.to_csv(LABELED_IMU_PATH, index=False)
    
    # 4. 统一特征工程 (滑动窗口)
    # 虽然合并处会有极微小的窗口误差(10行)，但在4万行数据面前可忽略不计
    print(">> 提取特征 & 标准化...")
    X, y, y_phase = preprocess_features(df_combined)
    X_scaled, scaler = standardize_data(X)
    
    # 5. 强制 SVM 训练 (带进度显示)
    print("\n>> 正在强制训练 SVM 模型 (这可能需要 5-10 分钟，请耐心等待)...")
    try:
        model, metrics = train_optimized_model(X_scaled, y, y_phase)
        
        # 只有在 SVM 极差时才切换
        if metrics['R²决定系数'] < 0.6:
            print("SVM 效果不佳，尝试随机森林...")
            model, metrics = train_fallback_model(X_scaled, y, y_phase)
            
    except Exception as e:
        print(f"SVM 训练意外中断: {e}")
        print("切换到随机森林作为备选...")
        model, metrics = train_fallback_model(X_scaled, y, y_phase)

    # 6. 验证与汇总
    verify_results()
    
    print(f"\n===== 最终模型评估汇总 =====")
    print(f"R²决定系数：{metrics['R²决定系数']:.4f}")
    print(f"步态分类准确率：{metrics['步态阶段分类准确率(%)']:.2f}%")