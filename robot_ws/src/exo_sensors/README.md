# exo_sensors - 外骨骼传感器数据采集包

## 📦 功能包概述

用于从 ESP32 IMU 传感器读取 CSV 格式数据，并发布到 ROS2 话题，支持数据记录和实时可视化。专为步态识别 SVM 训练设计。

## 🚀 快速开始

### 1. 编译功能包

```bash
cd robot_ws
colcon build --packages-select exo_sensors
source install/setup.bash
```

### 2. 配置串口

编辑 `config/imu_csv_reader.yaml`，修改串口名称:
- Windows: `COM3`, `COM4` 等
- Linux: `/dev/ttyUSB0`, `/dev/ttyACM0` 等

### 3. 运行节点

#### 方式 1: 单独运行 CSV 读取器
```bash
ros2 run exo_sensors imu_csv_reader
```

#### 方式 2: 使用 Launch 文件 (推荐)
```bash
# 启动完整系统 (读取器 + 记录器 + 可视化)
ros2 launch exo_sensors imu_complete_system.launch.py serial_port:=COM3

# 只启动读取器
ros2 launch exo_sensors imu_csv_reader.launch.py serial_port:=COM3
```

#### 方式 3: 分别运行各个节点
```bash
# 终端 1: 启动 CSV 读取器
ros2 run exo_sensors imu_csv_reader --ros-args -p serial_port:=COM3

# 终端 2: 启动数据记录器 (可选)
ros2 run exo_sensors imu_data_recorder

# 终端 3: 启动可视化器 (可选)
ros2 run exo_sensors imu_visualizer
```

## 📊 节点说明

### 1. `imu_csv_reader` - IMU CSV 读取器

**功能**: 从 ESP32 串口读取 CSV 格式的 IMU 数据并发布到 `/imu/data` 话题

**参数**:
- `serial_port` (string): 串口设备名，默认 `COM3`
- `baud_rate` (int): 波特率，默认 `115200`
- `topic_name` (string): 发布话题名，默认 `/imu/data`
- `auto_detect_port` (bool): 自动检测 ESP32 串口，默认 `false`

**发布话题**:
- `/imu/data` (sensor_msgs/Imu): IMU 数据 (姿态四元数 + 角速度 + 线性加速度)

**特性**:
- ✅ 自动过滤 `mg`, `dps` 等单位行
- ✅ 单位自动转换 (mg→m/s², dps→rad/s, 度→弧度)
- ✅ 欧拉角自动转四元数
- ✅ 实时统计显示 (成功率、错误率、姿态角)

---

### 2. `imu_data_recorder` - IMU 数据记录器

**功能**: 订阅 `/imu/data` 并保存为 CSV 文件，用于 SVM 训练

**参数**:
- `topic_name` (string): 订阅话题名，默认 `/imu/data`
- `output_dir` (string): 输出目录，默认 `./data`
- `file_prefix` (string): 文件名前缀，默认 `imu_gait_data`
- `auto_filename` (bool): 自动生成带时间戳文件名，默认 `true`

**输出格式** (CSV):
```csv
Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ
1234,1.23,-0.45,89.12,50.123,-30.456,1000.789,0.12,-0.34,0.56
```

---

### 3. `imu_visualizer` - IMU 可视化器

**功能**: 实时绘制 Roll/Pitch/Yaw 姿态角曲线

**参数**:
- `topic_name` (string): 订阅话题名，默认 `/imu/data`
- `window_size` (int): 显示数据点数量，默认 `200` (约10秒)

**特性**:
- 📈 三通道实时曲线 (Roll/Pitch/Yaw)
- 🎨 美观的 matplotlib 图表
- ⏱️ 自动滚动时间轴

## 🔧 配置文件

### `config/imu_csv_reader.yaml`
```yaml
imu_csv_reader:
  ros__parameters:
    serial_port: "COM3"          # 修改为你的串口
    baud_rate: 115200
    topic_name: "/imu/data"
    auto_detect_port: false
```

### `config/imu_data_recorder.yaml`
```yaml
imu_data_recorder:
  ros__parameters:
    output_dir: "./data"
    file_prefix: "imu_gait_data"
    auto_filename: true
```

## 📡 话题消息格式

### `/imu/data` (sensor_msgs/Imu)

```
header:
  stamp: {sec: 1234, nanosec: 567890000}
  frame_id: "imu_link"
orientation:                    # 姿态四元数
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
angular_velocity:               # 角速度 (rad/s)
  x: 0.12
  y: -0.34
  z: 0.56
linear_acceleration:            # 线性加速度 (m/s²)
  x: 0.49
  y: -0.29
  z: 9.81
```

## 🛠️ 依赖项

**ROS2 依赖**:
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`

**Python 依赖**:
```bash
pip install pyserial numpy matplotlib
```

## 📝 使用示例

### 示例 1: 采集步态数据训练 SVM

```bash
# 1. 启动读取器和记录器
ros2 launch exo_sensors imu_complete_system.launch.py \
  serial_port:=COM3 \
  enable_visualizer:=false

# 2. 让受试者行走 30 秒
# 3. Ctrl+C 停止，数据自动保存到 ./data/imu_gait_data_YYYYMMDD_HHMMSS.csv
```

### 示例 2: 实时监控 IMU 姿态

```bash
# 启动读取器和可视化器
ros2 launch exo_sensors imu_complete_system.launch.py \
  serial_port:=COM3 \
  enable_recorder:=false
```

### 示例 3: 查看实时数据

```bash
# 终端 1: 启动读取器
ros2 run exo_sensors imu_csv_reader --ros-args -p serial_port:=COM3

# 终端 2: 查看话题
ros2 topic echo /imu/data

# 终端 3: 查看频率
ros2 topic hz /imu/data
```

## 🐛 故障排除

### 问题 1: 找不到串口

**Linux 下**:
```bash
# 查看可用串口
ls /dev/tty*

# 添加用户到 dialout 组
sudo usermod -aG dialout $USER
# 重新登录生效
```

**Windows 下**:
- 打开设备管理器 → 端口 (COM和LPT) → 查看 COM 端口号

### 问题 2: 数据解析错误

检查 ESP32 输出格式是否为:
```csv
Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ
```

### 问题 3: 可视化窗口无响应

确保安装了 matplotlib 的 GUI 后端:
```bash
pip install PyQt5  # 或 pip install tkinter
```

## 📂 目录结构

```
exo_sensors/
├── config/                     # 配置文件
│   ├── imu_csv_reader.yaml
│   ├── imu_data_recorder.yaml
│   └── imu_visualizer.yaml
├── data/                       # 数据输出目录 (自动创建)
├── exo_sensors/               # Python 源代码
│   ├── __init__.py
│   ├── imu_csv_reader.py      # CSV 读取器
│   ├── imu_data_recorder.py   # 数据记录器
│   └── imu_visualizer.py      # 可视化器
├── launch/                     # Launch 文件
│   ├── imu_csv_reader.launch.py
│   └── imu_complete_system.launch.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 📄 许可证

MIT License

## 👤 作者

BUAA BME Training Project
