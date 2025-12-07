# IMU 串口配置使用指南

## 📁 配置文件说明

项目现在包含两套配置,适应不同的部署环境:

### 1. 开发环境 (Windows + WSL2 + Docker)
- **配置文件**: `imu_params_dev_wsl.yaml`
- **Launch 文件**: `imu_dev_wsl.launch.py`
- **串口设备**: `/dev/ttyUSB0` (通过 usbipd-win)
- **特点**: 需要 Windows 侧安装 usbipd-win 工具

### 2. 生产环境 (树莓派 + Docker)
- **配置文件**: `imu_params_raspberry_pi.yaml`
- **Launch 文件**: `imu_raspberry_pi.launch.py`
- **串口设备**: `/dev/ttyUSB0` (树莓派原生 USB)
- **特点**: 直接硬件访问,无需额外工具

---

## 🚀 快速启动

### 开发环境 (当前环境)

```bash
# 1. 确保 USB 设备已通过 usbipd 连接
# 在 Windows PowerShell 运行: usbipd attach --wsl --busid 1-4

# 2. 验证设备
ls -l /dev/ttyUSB0

# 3. Source 环境
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. 启动 IMU 节点
ros2 launch imu_serial_publisher imu_dev_wsl.launch.py

# 5. 在新终端查看数据
ros2 topic echo /imu/data
```

### 树莓派部署

```bash
# 1. 验证设备
ls -l /dev/ttyUSB0

# 2. Source 环境
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# 3. 启动 IMU 节点
ros2 launch imu_serial_publisher imu_raspberry_pi.launch.py

# 4. 在新终端查看数据
ros2 topic echo /imu/data
```

---

## 📝 配置文件位置

### 源码目录
```
robot_ws/src/imu_serial_publisher/
├── config/
│   ├── imu_params.yaml              # 默认配置
│   ├── imu_params_dev_wsl.yaml      # 开发环境配置
│   └── imu_params_raspberry_pi.yaml # 生产环境配置
└── launch/
    ├── imu_serial.launch.py         # 通用启动文件
    ├── imu_dev_wsl.launch.py        # 开发环境启动
    └── imu_raspberry_pi.launch.py   # 生产环境启动
```

### 安装目录 (编译后)
```
install/imu_serial_publisher/share/imu_serial_publisher/
├── config/
│   ├── imu_params.yaml
│   ├── imu_params_dev_wsl.yaml
│   └── imu_params_raspberry_pi.yaml
└── launch/
    ├── imu_serial.launch.py
    ├── imu_dev_wsl.launch.py
    └── imu_raspberry_pi.launch.py
```

---

## 🔧 自定义参数

### 方式1: 修改配置文件

编辑对应的 YAML 文件:

```yaml
# 开发环境: robot_ws/src/imu_serial_publisher/config/imu_params_dev_wsl.yaml
serial_port: "/dev/ttyUSB0"  # 修改串口设备
baud_rate: 115200             # 修改波特率
frame_id: "imu_link"
publish_rate: 20.0            # 修改发布频率
```

修改后重新编译:
```bash
cd /workspace
colcon build --packages-select imu_serial_publisher
```

### 方式2: Launch 参数覆盖

```bash
# 指定不同的串口
ros2 launch imu_serial_publisher imu_dev_wsl.launch.py serial_port:=/dev/ttyUSB1

# 指定不同的波特率
ros2 launch imu_serial_publisher imu_dev_wsl.launch.py baud_rate:=9600

# 同时指定多个参数
ros2 launch imu_serial_publisher imu_dev_wsl.launch.py \
  serial_port:=/dev/ttyACM0 \
  baud_rate:=115200
```

### 方式3: 直接运行节点并指定配置

```bash
# 使用指定的配置文件
ros2 run imu_serial_publisher imu_serial_publisher_node \
  --ros-args \
  --params-file /workspace/robot_ws/src/imu_serial_publisher/config/imu_params_dev_wsl.yaml
```

---

## 🧪 测试串口连接

### 测试1: 原始数据读取

```bash
# 配置串口参数
stty -F /dev/ttyUSB0 115200 raw -echo

# 读取3秒数据
timeout 3 cat /dev/ttyUSB0

# 应该看到 JSON 格式输出:
# {"orientation":{"roll":0.21,"pitch":-1.32,"yaw":-127.29},...}
```

### 测试2: ROS2 话题验证

```bash
# 启动节点后,在新终端运行:

# 查看话题列表
ros2 topic list
# 应该看到:
# /imu/data
# /imu/mag

# 查看话题信息
ros2 topic info /imu/data
# Type: sensor_msgs/msg/Imu

# 查看发布频率
ros2 topic hz /imu/data
# average rate: 20.000

# 查看一条消息
ros2 topic echo /imu/data --once
```

### 测试3: 数据可视化

```bash
# 安装 rqt 工具 (如果没有)
sudo apt install ros-humble-rqt*

# 启动 rqt 图形界面
rqt

# 在 rqt 中选择: Plugins → Visualization → Plot
# 添加话题: /imu/data/orientation/roll
```

---

## 📊 当前配置状态

### 开发环境 (WSL2)
- ✅ 串口设备: `/dev/ttyUSB0` (COM7 通过 usbipd)
- ✅ 波特率: `115200`
- ✅ 数据格式: JSON
- ✅ 配置文件: `imu_params_dev_wsl.yaml`
- ✅ 测试状态: 串口数据读取成功

### 树莓派 (待部署)
- ⏳ 串口设备: `/dev/ttyUSB0` (待确认)
- ⏳ 波特率: `115200`
- ⏳ 配置文件: `imu_params_raspberry_pi.yaml`
- ⏳ Docker 配置: 需要添加 `--device` 映射

---

## 🔄 切换环境流程

### 从开发切换到生产

```bash
# 1. 在树莓派上拉取最新代码
git pull origin master

# 2. 编译
cd /workspace
source /opt/ros/humble/setup.bash
colcon build

# 3. 查看树莓派的串口设备
ls -l /dev/ttyUSB*

# 4. 如果设备号不同,修改配置文件
vim robot_ws/src/imu_serial_publisher/config/imu_params_raspberry_pi.yaml

# 5. 重新编译
colcon build --packages-select imu_serial_publisher

# 6. 启动
source install/setup.bash
ros2 launch imu_serial_publisher imu_raspberry_pi.launch.py
```

### 从生产回到开发

```bash
# 1. 确保 Windows 上的 usbipd 已连接设备
# PowerShell: usbipd attach --wsl --busid 1-4

# 2. 验证设备
ls -l /dev/ttyUSB0

# 3. 启动开发配置
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch imu_serial_publisher imu_dev_wsl.launch.py
```

---

## 📖 相关文档

- [USB串口连接WSL2流程.md](./USB串口连接WSL2流程.md) - usbipd-win 详细配置
- [树莓派Docker部署配置.md](./树莓派Docker部署配置.md) - 树莓派部署完整指南
- [IMU通信协议说明](../../robot_ws/src/imu_serial_publisher/COMMUNICATION_EXPLAINED.md)

---

**更新日期:** 2025-12-05
