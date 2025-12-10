import pandas as pd
import numpy as np
import joblib
import os
import warnings
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

# ===================== 核心配置 =====================
BASE_DIR = "/home/buaaasus/butaishibie"
RAW_IMU_PATH = os.path.join(BASE_DIR, "imu_gait_data_20251208_161501.csv")
LABELED_IMU_PATH = os.path.join(BASE_DIR, "带步态标签的IMU数据.csv")
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
    print(f"[2/7] 生成步态标签（优化峰值检测）...")
    df = df_raw.copy()
    df['RelativeTime'] = (df['Timestamp'] - df['Timestamp'].min()) / 1000
    
    # 优化AccZ信号处理
    acc_z_median = df['AccZ'].median()
    acc_z_std = df['AccZ'].std()
    df['AccZ_clean'] = np.where(
        np.abs(df['AccZ'] - acc_z_median) > 3 * acc_z_std,
        acc_z_median,
        df['AccZ']
    )
    df['AccZ_detrended'] = detrend(df['AccZ_clean'].values)
    
    # 优化峰值检测参数
    peaks, _ = find_peaks(
        df['AccZ_detrended'],
        height=np.percentile(df['AccZ_detrended'], 60),  # 降低阈值
        distance=int(50 * 0.8),  # 减小峰间距
        prominence=0.5  # 增加突出度筛选
    )
    
    if len(peaks) < 2:
        gait_cycle = 1.2
        print(f"峰值检测失败，使用默认步态周期：{gait_cycle}秒")
    else:
        peak_times = df['RelativeTime'].iloc[peaks].values
        gait_cycles = np.diff(peak_times)
        gait_cycle = np.mean(gait_cycles)
        print(f"自动检测步态周期：{gait_cycle:.2f}秒（基于{len(peaks)}个峰值）")
    
    # 优化标签生成（降低噪声影响）
    df['TimeLabel'] = (df['RelativeTime'] % gait_cycle) / gait_cycle
    acc_z_norm = (df['AccZ_detrended'] - df['AccZ_detrended'].min()) / (df['AccZ_detrended'].max() - df['AccZ_detrended'].min())
    df['GaitCycleLabel'] = 0.85 * df['TimeLabel'] + 0.15 * acc_z_norm  # 降低AccZ权重
    df['GaitCycleLabel'] = (df['GaitCycleLabel'] - df['GaitCycleLabel'].min()) / (df['GaitCycleLabel'].max() - df['GaitCycleLabel'].min())
    df['GaitPhase'] = df['GaitCycleLabel'].apply(get_gait_phase)
    
    # 平衡数据分布
    df_balanced = balance_gait_phase_data(df)
    
    df_balanced.to_csv(LABELED_IMU_PATH, index=False)
    print(f"带标签数据已保存：{LABELED_IMU_PATH}")
    print(f"标签范围：{df_balanced['GaitCycleLabel'].min():.4f} ~ {df_balanced['GaitCycleLabel'].max():.4f}")
    return df_balanced

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
    
    # 网格搜索
    grid_search = GridSearchCV(
        estimator=SVR(),
        param_grid=param_grid,
        cv=3,  # 减少交叉验证折数，加快训练
        scoring='r2',
        n_jobs=-1,
        verbose=0
    )
    
    # 训练（加入样本权重）
    grid_search.fit(X_train, y_train, sample_weight=sample_weights)
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
    
    # 加载/生成数据
    try:
        df_raw = pd.read_csv(RAW_IMU_PATH)
        print(f"加载真实无标签数据：{RAW_IMU_PATH}，样本数：{len(df_raw)}")
    except FileNotFoundError:
        print(f"未找到真实数据，生成模拟数据...")
        df_raw = generate_raw_imu_data()
    
    # 生成标签（含平衡）
    df_labeled = generate_gait_labels(df_raw)
    
    # 特征工程
    X, y, y_phase = preprocess_features(df_labeled)
    
    # 标准化
    X_scaled, scaler = standardize_data(X)
    
    # 训练模型
    try:
        model, metrics = train_optimized_model(X_scaled, y, y_phase)
        # 若SVM R²仍<0.6，切换到随机森林
        if metrics['R²决定系数'] < 0.6:
            model, metrics = train_fallback_model(X_scaled, y, y_phase)
    except Exception as e:
        print(f"SVM训练失败：{e}，切换到随机森林...")
        model, metrics = train_fallback_model(X_scaled, y, y_phase)
    
    # 验证文件
    verify_results()
    
    # 最终汇总
    print(f"\n===== 最终模型评估汇总 =====")
    print(f"核心回归指标 - R²：{metrics['R²决定系数']:.4f}，RMSE：{metrics['RMSE均方根误差']:.4f}")
    print(f"核心准确率指标 - 步态阶段分类准确率：{metrics['步态阶段分类准确率(%)']:.2f}%")