# 自定义机器人控制项目

基于ESP32 IMU传感器和宇树电机的自定义机器人控制系统。

## 📁 项目结构

```
BUAA_BME_Training/
├── 1note/                          # 开发笔记
├── .devcontainer/                  # Docker开发环境配置
├── sensor_source_code/             # ESP32传感器源代码
│   └── 9DOF_Demo/                  # 9轴IMU示例代码
├── robot_ws/                       # ROS2工作空间 ⭐ 主要开发目录
│   ├── src/                        # C++源代码
│   │   ├── robot_controller/       # 机器人控制功能包
│   │   └── imu_serial_publisher/   # IMU串口发布器
│   └── scripts/                    # Python脚本
└── README.md
```

## 🚀 快速开始

### 1. 环境配置

```bash
# 安装ROS2 Humble
sudo apt install ros-humble-desktop

# 安装依赖
sudo apt install python3-colcon-common-extensions
```

### 2. 编译工作空间

```bash
cd robot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. 运行系统

**终端1 - 启动IMU发布器:**
```bash
ros2 run imu_serial_publisher imu_serial_publisher_node \
  --ros-args -p serial_port:=/dev/ttyUSB0
```

**终端2 - 启动机器人控制器:**
```bash
ros2 run robot_controller main_controller
```

## 📦 功能包说明

### imu_serial_publisher
- **功能**: 读取ESP32 IMU数据并发布到ROS2话题
- **话题**: `/imu/data` (sensor_msgs/Imu)
- **配置**: 串口、波特率、坐标系等

### robot_controller
- **功能**: 主控制逻辑
- **输入**: IMU数据
- **输出**: 电机控制命令
- **算法**: 待实现

## 🔧 开发指南

### 添加新的控制算法

编辑 `robot_ws/src/robot_controller/src/main_controller.cpp`:

```cpp
void control_algorithm() {
    // 在这里实现你的算法
}
```

### 添加Python脚本

将脚本放在 `robot_ws/scripts/` 目录:

```bash
chmod +x robot_ws/scripts/your_script.py
ros2 run robot_controller your_script.py
```

## 📝 备份说明

- 宇树功能包已备份至 `backup_12.5` 分支
- 如需恢复: `git checkout backup_12.5`

## 📚 文档

- [IMU通信原理](robot_ws/src/imu_serial_publisher/COMMUNICATION_EXPLAINED.md)
- [IMU使用指南](robot_ws/src/imu_serial_publisher/README.md)

## 许可证

Apache-2.0
