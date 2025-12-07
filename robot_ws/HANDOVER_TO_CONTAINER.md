# 容器内 ROS2 工作交接说明

## 📋 当前状态

### 1. 工作环境
- **宿主机**: Windows 系统，PowerShell 终端
- **ROS2 环境**: 需要在 Docker 容器内操作
- **工作目录**: `d:\1CodeProject\BUAA_BME_Training\robot_ws`

### 2. 已完成的工作

#### ✅ ESP32 IMU 代码 (已完成)
- **位置**: `sensor_source_code/9DOF_Demo_v3/`
- **主程序**: `9DOF_Demo_v3.ino`
- **功能**: 
  - 100Hz (10ms间隔) 非阻塞采样
  - 输出 CSV 格式: `Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ`
  - 使用 `snprintf()` 优化，减少串口阻塞
- **数据单位**:
  - Roll/Pitch/Yaw: 度 (°)
  - AccX/Y/Z: mg (毫重力加速度)
  - GyroX/Y/Z: dps (度每秒)
- **已知问题**: QMI8658.cpp 会打印 `mg` 和 `dps` 单位行，但已保留（注释掉会导致编译错误）

#### ✅ ROS2 功能包框架 (已创建文件结构)
- **包名**: `exo_sensors`
- **位置**: `robot_ws/src/exo_sensors/`
- **已创建文件**:
  ```
  exo_sensors/
  ├── config/
  │   ├── imu_csv_reader.yaml          # CSV读取器配置
  │   ├── imu_data_recorder.yaml       # 数据记录器配置
  │   └── imu_visualizer.yaml          # 可视化器配置
  ├── exo_sensors/
  │   ├── __init__.py
  │   ├── imu_csv_reader.py            # 主节点：串口CSV读取+过滤单位行
  │   ├── imu_data_recorder.py         # 数据记录节点
  │   └── imu_visualizer.py            # 实时可视化节点
  ├── launch/
  │   ├── imu_csv_reader.launch.py
  │   └── imu_complete_system.launch.py
  ├── resource/
  │   └── exo_sensors
  ├── data/                             # 数据输出目录
  ├── package.xml
  ├── setup.py
  ├── setup.cfg
  └── README.md
  ```

### 3. 核心需求

#### 🎯 主要目标
为外骨骼项目创建 IMU 数据采集系统，用于**步态识别 SVM 训练**

#### 📡 数据流
```
ESP32 IMU (CSV) 
  → 串口 (COM3/ttyUSB0) 
  → imu_csv_reader 节点 (过滤单位行 + 转换单位) 
  → /imu/data 话题 (sensor_msgs/Imu)
  → 记录器/可视化器
```

#### 🔧 关键功能
1. **CSV 读取器** (`imu_csv_reader.py`):
   - 从串口读取 CSV 数据
   - **过滤掉** `mg`, `dps`, `m/s2`, `rad/s` 等单位行
   - 过滤掉 CSV 表头行
   - 单位转换:
     - 角度 → 弧度 → 四元数
     - mg → m/s²
     - dps → rad/s
   - 发布到 `/imu/data` (sensor_msgs/Imu)

2. **数据记录器** (`imu_data_recorder.py`):
   - 订阅 `/imu/data`
   - 保存为 CSV 格式（与原始格式一致）
   - 自动生成带时间戳的文件名

3. **可视化器** (`imu_visualizer.py`):
   - 实时绘制 Roll/Pitch/Yaw 曲线
   - matplotlib 三通道图表

## 🚀 需要完成的任务

### 任务清单

#### 1. 编译功能包
```bash
cd /workspace/robot_ws  # 容器内路径
colcon build --packages-select exo_sensors
source install/setup.bash
```

#### 2. 测试功能包
```bash
# 测试 1: 检查节点是否注册
ros2 pkg list | grep exo_sensors

# 测试 2: 查看可执行文件
ros2 pkg executables exo_sensors
# 预期输出:
# exo_sensors imu_csv_reader
# exo_sensors imu_data_recorder
# exo_sensors imu_visualizer

# 测试 3: 检查参数
ros2 run exo_sensors imu_csv_reader --ros-args --show-args
```

#### 3. 修复可能的编译错误
**常见问题**:
- Python 依赖缺失: `pip install pyserial numpy matplotlib`
- launch 文件的 `condition` 语法问题 (ROS2 版本差异)
- 文件权限问题: `chmod +x launch/*.py`

#### 4. 配置串口参数
编辑 `config/imu_csv_reader.yaml`:
```yaml
serial_port: "/dev/ttyUSB0"  # Linux 下修改为实际串口
```

#### 5. 运行测试（如果有 ESP32 连接）
```bash
# 方式 1: 单独节点
ros2 run exo_sensors imu_csv_reader --ros-args -p serial_port:=/dev/ttyUSB0

# 方式 2: Launch 文件
ros2 launch exo_sensors imu_csv_reader.launch.py serial_port:=/dev/ttyUSB0

# 方式 3: 完整系统
ros2 launch exo_sensors imu_complete_system.launch.py serial_port:=/dev/ttyUSB0
```

## 📝 重要注意事项

### ⚠️ 必须处理的数据过滤
ESP32 输出示例:
```
Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ
mg          ← 必须过滤
dps         ← 必须过滤
1234,1.23,-0.45,89.12,50.123,-30.456,1000.789,0.12,-0.34,0.56
mg          ← 必须过滤
dps         ← 必须过滤
1244,1.25,-0.43,89.15,52.134,-29.345,999.876,0.11,-0.33,0.57
```

**过滤逻辑** (已在 `imu_csv_reader.py` 中实现):
```python
if not line or line in ['mg', 'dps', 'm/s2', 'rad/s', 'Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ']:
    return  # 跳过单位行和表头
```

### 🔢 单位转换公式 (已实现)
```python
# 角度 → 弧度
roll_rad = math.radians(roll_deg)

# mg → m/s²
acc_ms2 = acc_mg * 0.00980665  # 1mg = 0.00980665 m/s²

# dps → rad/s
gyro_rads = math.radians(gyro_dps)

# 欧拉角 → 四元数 (使用 euler_to_quaternion 函数)
```

### 📦 依赖检查
```bash
# 在容器内检查
python3 -c "import serial; import numpy; import matplotlib; print('✅ 依赖已安装')"

# 如果缺失，安装:
pip3 install pyserial numpy matplotlib
```

## 🐛 可能遇到的问题

### 问题 1: Launch 文件 condition 语法错误
如果 ROS2 版本 < Humble，需要修改:
```python
# 旧写法 (可能报错)
condition=lambda context: context.launch_configurations['enable_recorder'] == 'true'

# 新写法
from launch.conditions import IfCondition
condition=IfCondition(LaunchConfiguration('enable_recorder'))
```

### 问题 2: 串口权限
```bash
sudo chmod 666 /dev/ttyUSB0
# 或
sudo usermod -aG dialout $USER  # 需重新登录
```

### 问题 3: matplotlib 后端
如果可视化无法显示:
```python
# 在 imu_visualizer.py 开头添加
import matplotlib
matplotlib.use('TkAgg')  # 或 'Qt5Agg'
```

## ✅ 验收标准

完成以下测试即为成功:

1. ✅ `colcon build` 无错误
2. ✅ `ros2 pkg executables exo_sensors` 显示 3 个节点
3. ✅ `ros2 run exo_sensors imu_csv_reader` 能启动（无串口时显示错误是正常的）
4. ✅ 如果有 ESP32: 能正确解析 CSV 数据并发布到 `/imu/data`
5. ✅ `ros2 topic echo /imu/data` 能看到消息

## 📂 相关文件路径

- **ESP32 代码**: `/workspace/sensor_source_code/9DOF_Demo_v3/9DOF_Demo_v3.ino`
- **ROS2 包**: `/workspace/robot_ws/src/exo_sensors/`
- **配置文件**: `/workspace/robot_ws/src/exo_sensors/config/*.yaml`
- **Python 节点**: `/workspace/robot_ws/src/exo_sensors/exo_sensors/*.py`

## 🎯 最终目标

创建一个稳定的 IMU 数据采集系统:
- 能从 ESP32 正确读取 100Hz 的 CSV 数据
- 自动过滤调试信息 (mg/dps 行)
- 转换为 ROS2 标准格式
- 支持数据记录和可视化
- 为后续 SVM 步态识别提供干净的训练数据

---

## 💬 交接完成

如有问题，检查:
1. `exo_sensors/README.md` - 详细使用文档
2. `exo_sensors/exo_sensors/imu_csv_reader.py` - 核心逻辑（第62-66行是过滤逻辑）
3. Python 依赖是否安装完整

Good luck! 🚀
