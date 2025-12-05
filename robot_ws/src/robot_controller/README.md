# 机器人控制器使用指南

## 功能包说明

### robot_controller
主控制器功能包，负责读取传感器数据并运行控制算法。

**主要文件**:
- `src/main_controller.cpp` - 主控制逻辑 (C++)
- `config/controller_params.yaml` - 参数配置
- `launch/robot_system.launch.py` - 启动文件

## 快速开始

### 1. 编译工作空间

```bash
cd ~/BUAA_BME_Training/robot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. 运行系统

**方法1: 使用launch文件 (推荐)**
```bash
# 监控模式 (不使能控制)
ros2 launch robot_controller robot_system.launch.py

# 启用控制
ros2 launch robot_controller robot_system.launch.py enable_control:=true

# 指定串口
ros2 launch robot_controller robot_system.launch.py serial_port:=/dev/ttyACM0
```

**方法2: 分别启动节点**
```bash
# 终端1 - IMU发布器
ros2 run imu_serial_publisher imu_serial_publisher_node \
  --ros-args -p serial_port:=/dev/ttyUSB0

# 终端2 - 控制器
ros2 run robot_controller main_controller \
  --ros-args -p enable_control:=false
```

### 3. 数据记录和可视化

**记录数据到CSV:**
```bash
ros2 run robot_controller data_logger.py
# 数据保存在 robot_data/ 目录
```

**实时可视化IMU数据:**
```bash
# 需要先安装: pip install matplotlib
ros2 run robot_controller visualization.py
```

## 参数配置

编辑 `config/controller_params.yaml`:

```yaml
robot_controller:
  ros__parameters:
    control_frequency: 50.0        # 控制频率
    enable_control: false          # 控制使能
    num_motors: 12                 # 电机数量
    imu_topic: "/imu/data"        # IMU话题
```

## 开发指南

### 实现控制算法

编辑 `src/main_controller.cpp`，找到 `run_control_algorithm()` 函数:

```cpp
void run_control_algorithm()
{
    // 1. 读取传感器数据
    double roll = imu_roll_;
    double pitch = imu_pitch_;
    
    // 2. 实现你的算法
    // 例如: 平衡控制
    double roll_error = 0.0 - roll;
    double kp = 50.0;
    
    // 3. 计算电机命令
    for (int i = 0; i < num_motors_; i++) {
        motor_commands_[i] = kp * roll_error;
    }
}
```

### 实现电机命令发布

编辑 `publish_motor_commands()` 函数，根据你的电机接口发布命令。

### 添加新的传感器

1. 在构造函数中添加订阅:
```cpp
laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&RobotController::laser_callback, this, std::placeholders::_1));
```

2. 实现回调函数:
```cpp
void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // 处理激光数据
}
```

## 话题列表

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/imu/data` | sensor_msgs/Imu | IMU传感器数据 |
| `/motor_commands` | std_msgs/Float32MultiArray | 电机命令 |
| `/cmd_vel` | geometry_msgs/Twist | 速度命令 (可选) |

## 安全注意事项

⚠️ **重要**: 
1. 首次运行时保持 `enable_control:=false`
2. 确保机器人处于安全环境
3. 准备好急停开关
4. 逐步调试控制参数

## 故障排查

### 问题1: 收不到IMU数据
```bash
# 检查串口设备
ls /dev/tty*

# 检查话题
ros2 topic list | grep imu

# 检查IMU节点日志
ros2 run imu_serial_publisher imu_serial_publisher_node --ros-args --log-level debug
```

### 问题2: 控制器不响应
- 确认 `enable_control` 参数为 `true`
- 检查传感器数据是否正常接收
- 查看控制器日志输出

### 问题3: 编译错误
```bash
# 清理并重新编译
cd robot_ws
rm -rf build install log
colcon build --symlink-install
```

## 进阶功能

### 1. 使用RViz2可视化
```bash
ros2 run rviz2 rviz2
# 添加IMU显示，设置Fixed Frame为 imu_link
```

### 2. 录制rosbag
```bash
ros2 bag record /imu/data /motor_commands
```

### 3. 回放数据
```bash
ros2 bag play <bag_file>
```

## 许可证

Apache-2.0
