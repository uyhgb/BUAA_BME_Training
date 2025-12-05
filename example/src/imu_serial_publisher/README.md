# ESP32 IMU串口数据发布器

将ESP32 IMU传感器数据通过串口读取并发布到ROS2话题。

## 🔑 核心原理

### 为什么使用JSON而不是直接调用函数?

```
┌─────────────┐     USB串口线      ┌──────────────┐
│   ESP32     │ ◄──────────────► │   电脑/ROS2   │
│  (硬件端)    │   JSON文本传输    │   (软件端)    │
└─────────────┘                  └──────────────┘
```

**关键理解**: 
- `imuDataGet()` 函数运行在 **ESP32微控制器**上
- ROS2节点运行在 **你的电脑**上
- 它们是两个**物理隔离的设备**,无法直接函数调用
- 必须通过**串口通信**传输数据

**JSON的作用**:
1. **编码**: ESP32将IMU数据编码成JSON字符串
2. **传输**: 通过串口发送文本数据
3. **解码**: 电脑端ROS2节点解析JSON并提取数据
4. **发布**: 转换为ROS2消息发布到话题

这就像两个人通过对讲机说话,需要把想法(数据)转成语言(JSON)才能传递!

## 功能特性

- 从ESP32串口读取JSON格式的IMU数据
- 发布标准ROS2 `sensor_msgs/Imu` 消息到 `/imu/data` 话题
- ~~发布磁力计数据到 `/imu/mag` 话题~~ (已禁用,如需使用见下文)
- 自动将欧拉角转换为四元数
- 支持配置串口、波特率和坐标系

## 硬件要求

- ESP32开发板 + 6DOF IMU传感器 (加速度计 + 陀螺仪)
- USB串口连接到运行ROS2的计算机

## ESP32端设置

### 1. 安装ArduinoJson库

在Arduino IDE中:
1. 打开 **工具 -> 管理库**
2. 搜索 "ArduinoJson"
3. 安装 `ArduinoJson by Benoit Blanchon` (推荐版本 6.x)

### 2. 上传代码

1. 打开 `sensor_source_code/9DOF_Demo/9DOF_Demo.ino`
2. 连接ESP32到电脑
3. 选择正确的开发板和端口
4. 上传代码

### 3. 数据格式

ESP32通过串口发送JSON格式数据 (115200波特率, 20Hz):

```json
{
  "orientation": {"roll": 0.12, "pitch": -0.45, "yaw": 90.3},
  "acceleration": {"x": 0.05, "y": -0.02, "z": 9.81},
  "gyroscope": {"x": 0.001, "y": -0.003, "z": 0.002},
  "timestamp": 123456
}
```

> **注意**: 磁力计数据已禁用。如需使用9DOF(包含磁力计),请查看"启用磁力计"章节。

## ROS2端设置

### 1. 编译功能包

```bash
cd ~/BUAA_BME_Training/example
source /opt/ros/humble/setup.bash  # 或你的ROS2版本
colcon build --packages-select imu_serial_publisher
source install/setup.bash
```

### 2. 配置串口权限 (Linux)

```bash
# 查找ESP32串口设备 (通常是 /dev/ttyUSB0 或 /dev/ttyACM0)
ls /dev/tty*

# 添加当前用户到dialout组 (一次性操作)
sudo usermod -a -G dialout $USER

# 或临时修改权限
sudo chmod 666 /dev/ttyUSB0
```

### 3. 运行节点

```bash
# 使用默认参数 (串口: /dev/ttyUSB0, 波特率: 115200)
ros2 run imu_serial_publisher imu_serial_publisher_node

# 自定义参数
ros2 run imu_serial_publisher imu_serial_publisher_node \
  --ros-args \
  -p serial_port:=/dev/ttyACM0 \
  -p baud_rate:=115200 \
  -p frame_id:=imu_link \
  -p publish_rate:=20.0
```

### 4. 查看数据

```bash
# 查看IMU话题
ros2 topic echo /imu/data

# 查看话题列表
ros2 topic list

# 查看话题频率
ros2 topic hz /imu/data
```

## 启用磁力计 (可选)

如果你的传感器有9DOF(包含磁力计)并想使用:

### ESP32端
1. 编辑 `IMU.cpp`,取消 `imuInit()` 中磁力计初始化的注释
2. 取消 `imuDataGet()` 中磁力计读取的注释
3. 编辑 `9DOF_Demo.ino`,取消磁力计JSON数据的注释
4. 重新上传代码

### ROS2端
1. 编辑 `imu_serial_publisher.cpp`,取消磁力计发布器和发布代码的注释
2. 重新编译: `colcon build --packages-select imu_serial_publisher`

发布的话题:
- `/imu/mag` (sensor_msgs/MagneticField) - 磁场强度

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `serial_port` | string | `/dev/ttyUSB0` | ESP32串口设备路径 |
| `baud_rate` | int | `115200` | 串口波特率 |
| `frame_id` | string | `imu_link` | TF坐标系名称 |
| `publish_rate` | double | `20.0` | 数据发布频率 (Hz) |

## 话题说明

### `/imu/data` (sensor_msgs/Imu)

包含:
- `orientation`: 四元数姿态 (从欧拉角转换)
- `angular_velocity`: 角速度 (rad/s)
- `linear_acceleration`: 线性加速度 (m/s²)

## 故障排除

### 1. 找不到串口设备

```bash
# 检查USB连接
lsusb

# 检查串口设备
dmesg | grep tty
```

### 2. 串口权限错误

```bash
# 重新登录或重启使组权限生效
# 或使用sudo运行 (不推荐)
sudo ros2 run imu_serial_publisher imu_serial_publisher_node
```

### 3. 数据解析错误

- 确认ESP32代码已正确上传
- 使用串口监视器检查JSON格式是否正确
- 检查波特率是否匹配

```bash
# 使用screen查看原始串口数据
screen /dev/ttyUSB0 115200

# 按 Ctrl-A 然后 K 退出screen
```

### 4. Windows系统

Windows用户需要修改代码中的串口读取部分,因为Linux的termios API在Windows上不可用。建议:
- 使用WSL2运行ROS2
- 或使用serial库替代 (需要额外依赖)

## 可视化

使用RViz2可视化IMU数据:

```bash
rviz2
```

在RViz2中:
1. 设置Fixed Frame为 `imu_link`
2. 添加 **Imu** 显示类型
3. 设置话题为 `/imu/data`

## 集成到launch文件

创建 `launch/imu_serial.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_serial_publisher',
            executable='imu_serial_publisher_node',
            name='imu_serial_publisher',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'frame_id': 'imu_link',
                'publish_rate': 20.0
            }]
        )
    ])
```

运行:
```bash
ros2 launch imu_serial_publisher imu_serial.launch.py
```

## 进阶: 数据校准

IMU数据可能需要校准。可以:
1. 修改ESP32代码添加偏置补偿
2. 使用ROS2的 `imu_filter_madgwick` 或 `robot_localization` 进行滤波
3. 使用 `imu_tools` 包进行校准

## 注意事项

- 确保IMU的坐标系与ROS REP-103标准一致
- 加速度应包含重力加速度 (9.81 m/s²)
- 磁力计数据可能需要硬铁/软铁校准
- 高频率发布可能增加CPU负载

## 许可证

Apache-2.0

## 作者

您的名字
