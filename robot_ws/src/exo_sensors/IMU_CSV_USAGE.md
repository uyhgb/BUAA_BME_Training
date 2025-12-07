# exo_sensors IMU CSV 数据采集使用指南

## 📋 概述

**功能**: 从 ESP32 (9DOF_Demo_v3) 读取 CSV 格式 IMU 数据并发布到 ROS2
**数据格式**: `Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ`
**采样率**: 100Hz (ESP32 端)
**应用**: 外骨骼步态识别的数据采集

---

## 🚀 快速启动

### 开发环境 (Windows + WSL2 + Docker)

**前置条件**:
```bash
# 1. Windows 端使用 usbipd 绑定并共享 USB 设备
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>

# 2. WSL2/Docker 内验证设备
ls -l /dev/ttyUSB*   # 应该看到 /dev/ttyUSB0
```

**启动命令**:
```bash
# 方式 1: 使用开发环境 launch 文件 (推荐)
source /opt/ros/humble/setup.bash
source /workspace/robot_ws/install/setup.bash
ros2 launch exo_sensors imu_csv_dev_wsl.launch.py

# 方式 2: 指定串口
ros2 launch exo_sensors imu_csv_dev_wsl.launch.py serial_port:=/dev/ttyUSB0

# 方式 3: 单节点运行
ros2 run exo_sensors imu_csv_reader --ros-args -p serial_port:=/dev/ttyUSB0
```

---

### 生产环境 (树莓派 + Docker)

**前置条件**:
```bash
# 1. 检查串口设备
ls -l /dev/ttyUSB* /dev/ttyACM*

# 2. 查看固定设备 ID (推荐)
ls -l /dev/serial/by-id/

# 3. 配置权限
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0  # 临时方案
```

**启动命令**:
```bash
# 方式 1: 使用树莓派 launch 文件 (推荐)
source /opt/ros/humble/setup.bash
source /workspace/robot_ws/install/setup.bash
ros2 launch exo_sensors imu_csv_raspberry_pi.launch.py

# 方式 2: 自动检测串口
ros2 launch exo_sensors imu_csv_raspberry_pi.launch.py auto_detect_port:=true

# 方式 3: Docker Compose
cd ~/robot_project
docker-compose up -d
```

---

## 📁 配置文件对比

| 参数 | 开发环境 (WSL2) | 生产环境 (树莓派) |
|------|----------------|------------------|
| **配置文件** | `imu_csv_dev_wsl.yaml` | `imu_csv_raspberry_pi.yaml` |
| **Launch文件** | `imu_csv_dev_wsl.launch.py` | `imu_csv_raspberry_pi.launch.py` |
| **串口设备** | `/dev/ttyUSB0` (usbipd映射) | `/dev/ttyUSB0` (原生USB) |
| **自动检测** | `false` (手动指定) | `true` (自动搜索) |
| **适用场景** | Windows开发调试 | 实际部署运行 |

---

## 🔍 数据验证

### 检查话题发布
```bash
# 1. 查看话题列表
ros2 topic list | grep imu

# 2. 查看一条消息
ros2 topic echo /imu/data --once

# 3. 检查发布频率
ros2 topic hz /imu/data
# 预期: average rate: ~100 Hz (取决于ESP32实际输出)
```

### 数据格式验证
```bash
# 查看完整消息结构
ros2 topic echo /imu/data --once --no-arr

# 预期输出:
# header:
#   stamp: {sec: xxx, nanosec: xxx}
#   frame_id: imu_link
# orientation: {x: xxx, y: xxx, z: xxx, w: xxx}  # 四元数
# angular_velocity: {x: xxx, y: xxx, z: xxx}     # rad/s
# linear_acceleration: {x: xxx, y: xxx, z: xxx}  # m/s²
```

---

## ⚙️ 配置参数说明

### 串口参数
- `serial_port`: 串口设备路径
  - WSL2: `/dev/ttyUSB0` (usbipd映射)
  - 树莓派: `/dev/ttyUSB0` 或 `/dev/serial/by-id/usb-xxx`
- `baud_rate`: 波特率 (必须与ESP32匹配: 115200)
- `timeout`: 读取超时时间 (秒)

### 功能参数
- `auto_detect_port`: 自动检测ESP32串口
  - `true`: 自动搜索 CP210x/CH340 芯片
  - `false`: 使用 serial_port 指定的设备
- `topic_name`: IMU数据发布话题 (默认: `/imu/data`)
- `frame_id`: 坐标系ID (默认: `imu_link`)

### 监控参数
- `enable_statistics`: 启用统计信息 (默认: `true`)
- `log_interval`: 统计输出间隔 (默认: 10秒)

---

## 🐛 故障排查

### 问题1: 找不到串口设备

**现象**: `could not open port /dev/ttyUSB0: No such file or directory`

**解决**:
```bash
# WSL2 环境
# 1. Windows PowerShell 检查设备
usbipd list
# 2. 如果设备未共享
usbipd attach --wsl --busid <BUSID>

# 树莓派环境
# 1. 检查 USB 设备
lsusb | grep -E "CP210|CH340|Silicon"
# 2. 检查内核日志
dmesg | grep tty | tail -10
# 3. 加载驱动
sudo modprobe cp210x
```

### 问题2: 串口权限不足

**现象**: `Permission denied: '/dev/ttyUSB0'`

**解决**:
```bash
# 临时方案
sudo chmod 666 /dev/ttyUSB0

# 永久方案
sudo usermod -a -G dialout $USER
# 然后重新登录或重启

# Docker 容器需要 privileged 或设备映射
docker run --device=/dev/ttyUSB0:/dev/ttyUSB0 ...
```

### 问题3: 数据解析错误

**现象**: `JSON parse error` 或 `CSV parse error`

**检查**:
```bash
# 1. 直接查看串口原始数据
cat /dev/ttyUSB0

# 2. 验证ESP32输出格式
# 应该看到:
# Timestamp,Roll,Pitch,Yaw,AccX,AccY,AccZ,GyroX,GyroY,GyroZ
# mg
# dps
# 1234,1.23,-0.45,89.12,50.123,-30.456,1000.789,0.12,-0.34,0.56
# mg
# dps

# 3. 检查波特率是否匹配
stty -F /dev/ttyUSB0
# 应显示 speed 115200 baud
```

### 问题4: 发布频率过低

**现象**: `ros2 topic hz /imu/data` 显示 < 50 Hz

**原因**: 
- ESP32 delay 设置过大
- 串口缓冲区积压
- 数据解析耗时过长

**解决**:
```bash
# 1. 检查 ESP32 代码中的 delay
# 9DOF_Demo_v3.ino 应该是 delay(10) for 100Hz

# 2. 清空串口缓冲
# 节点会自动清空,但可以重启节点

# 3. 查看节点日志
ros2 run exo_sensors imu_csv_reader
# 观察是否有大量过滤掉的行
```

---

## 📊 完整系统启动 (含记录和可视化)

```bash
# 启动完整系统
ros2 launch exo_sensors imu_complete_system.launch.py \
  serial_port:=/dev/ttyUSB0 \
  enable_recorder:=true \
  enable_visualizer:=true

# 数据保存位置
ls -lh /workspace/robot_ws/src/exo_sensors/data/
# 文件格式: imu_data_YYYYMMDD_HHMMSS.csv
```

---

## 📖 相关文件

- **ESP32 代码**: `/workspace/sensor_source_code/9DOF_Demo_v3/9DOF_Demo_v3.ino`
- **ROS2 节点**: `/workspace/robot_ws/src/exo_sensors/exo_sensors/imu_csv_reader.py`
- **配置文件**: `/workspace/robot_ws/src/exo_sensors/config/imu_csv_*.yaml`
- **Launch 文件**: `/workspace/robot_ws/src/exo_sensors/launch/imu_csv_*.launch.py`

---

**最后更新**: 2025-12-06
**版本**: v1.0.0
**维护**: exo_sensors package
