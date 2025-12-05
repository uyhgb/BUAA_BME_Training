# 自定义机器人桥接包

用自己的传感器 + 宇树电机搭建机器人的完整方案。

## 🎯 设计思路

```
你的传感器           宇树电机系统
(标准ROS2)    →→→   (宇树格式)
    ↓                    ↓
┌─────────┐        ┌──────────┐
│ ESP32   │        │ 宇树电机  │
│  IMU    │───┐    │ (CAN总线)│
└─────────┘   │    └──────────┘
              ↓           ↑
┌─────────┐   │           │
│ 编码器  │───┤    [控制命令]
└─────────┘   │           │
              ↓           │
┌─────────┐   │    ┌──────────────┐
│ 力传感器│───┴───→│ 桥接节点      │
└─────────┘        │ sensor_bridge │
                   └──────────────┘
                          ↓
                   /custom/lowstate
                   (模拟宇树格式)
                          ↓
                   ┌──────────────┐
                   │ 控制算法节点  │
                   │ controller   │
                   └──────────────┘
                          ↓
                      /lowcmd
                   (宇树电机命令)
```

## 📦 包含节点

### 1. `sensor_bridge_node` - 传感器桥接节点

**功能**: 将标准ROS2传感器消息转换为宇树LowState格式

**订阅话题**:
- `/imu/data` (sensor_msgs/Imu) - 你的ESP32 IMU
- `/joint_states` (sensor_msgs/JointState) - 关节编码器
- 可扩展更多传感器...

**发布话题**:
- `/custom/lowstate` (unitree_go/LowState) - 模拟宇树格式的状态

### 2. `custom_robot_controller` - 自定义控制器节点

**功能**: 基于传感器数据运行控制算法，输出电机命令

**订阅话题**:
- `/custom/lowstate` (unitree_go/LowState) - 来自桥接节点

**发布话题**:
- `/lowcmd` (unitree_go/LowCmd) - 宇树电机命令

**内置算法示例**:
- 姿态平衡控制
- 位置控制
- 力控制接口

## 🚀 快速开始

### 1. 编译

```bash
cd ~/BUAA_BME_Training/example
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash  # 宇树消息定义

colcon build --packages-select custom_robot_bridge
source install/setup.bash
```

### 2. 运行完整系统

**终端1**: 启动你的IMU发布器
```bash
ros2 run imu_serial_publisher imu_serial_publisher_node \
  --ros-args -p serial_port:=/dev/ttyUSB0
```

**终端2**: 启动传感器桥接节点
```bash
ros2 run custom_robot_bridge sensor_bridge_node
```

**终端3**: 启动控制器 (先不使能，仅监控)
```bash
ros2 run custom_robot_bridge custom_robot_controller \
  --ros-args -p enable_control:=false
```

**终端4**: 查看数据流
```bash
# 查看自定义lowstate
ros2 topic echo /custom/lowstate

# 查看电机命令
ros2 topic echo /lowcmd
```

### 3. 启用控制 (⚠️ 确保机器人安全后再执行)

```bash
ros2 run custom_robot_bridge custom_robot_controller \
  --ros-args \
  -p enable_control:=true \
  -p num_motors:=12 \
  -p control_frequency:=50.0
```

## 🔧 配置参数

### sensor_bridge_node
无特殊参数，自动桥接所有接收到的传感器数据。

### custom_robot_controller

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `num_motors` | int | 12 | 电机数量 |
| `control_frequency` | double | 50.0 | 控制频率 (Hz) |
| `enable_control` | bool | false | 控制使能 (安全开关) |

## 📝 自定义你的控制算法

编辑 `src/custom_robot_controller.cpp`，在 `balance_control()` 函数中实现你的算法:

```cpp
void balance_control()
{
    // 1. 读取传感器数据
    double roll = imu_roll_;
    double pitch = imu_pitch_;
    
    // 2. 运行你的算法
    // - 步态生成
    // - 状态机
    // - MPC控制
    // - 强化学习策略
    // ... 任何算法
    
    // 3. 输出电机命令
    for (int i = 0; i < num_motors_; i++) {
        lowcmd_msg_.motor_cmd[i].mode = 0x01;  // FOC模式
        lowcmd_msg_.motor_cmd[i].q = target_position[i];
        lowcmd_msg_.motor_cmd[i].kp = 50.0;
        lowcmd_msg_.motor_cmd[i].kd = 5.0;
    }
}
```

## 🔌 添加更多传感器

### 例如添加激光雷达:

```cpp
// 在 sensor_bridge_node.cpp 中添加
#include <sensor_msgs/msg/laser_scan.hpp>

// 构造函数中添加订阅
lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&SensorBridgeNode::lidar_callback, this, std::placeholders::_1));

// 添加回调函数
void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 处理激光数据，例如障碍物检测
    // 可以添加到LowState的自定义字段，或发布额外话题
}
```

## ⚠️ 安全注意事项

1. **测试阶段**: 始终将 `enable_control` 设为 `false`
2. **首次运行**: 确保机器人悬空或在安全环境
3. **增益调试**: 从小的 kp/kd 开始逐步增加
4. **急停准备**: 随时准备按下急停按钮
5. **电机温度**: 监控 `motor_state[i].temperature`

## 📊 数据流图

```
ESP32 IMU ──→ /imu/data (sensor_msgs/Imu)
                    ↓
            sensor_bridge_node
                    ↓
         /custom/lowstate (unitree_go/LowState)
                    ↓
          custom_robot_controller
                    ↓
              /lowcmd (unitree_go/LowCmd)
                    ↓
               宇树电机驱动
```

## 🎓 进阶功能

### 1. 添加状态估计器
```bash
# 使用robot_localization融合多传感器
sudo apt install ros-humble-robot-localization
```

### 2. 添加仿真支持
在Gazebo中模拟你的自定义机器人

### 3. 添加可视化
在RViz2中显示机器人状态和传感器数据

## 🐛 故障排查

### 问题1: 收不到lowstate数据
```bash
# 检查传感器是否发布
ros2 topic list | grep imu

# 检查桥接节点是否运行
ros2 node list
```

### 问题2: 电机不响应
- 确保 `enable_control=true`
- 检查电机模式是否为 `0x01` (FOC)
- 检查kp/kd是否合理

### 问题3: 控制不稳定
- 降低控制增益
- 提高控制频率
- 检查IMU数据质量

## 📚 参考资料

- [宇树电机文档](https://support.unitree.com)
- [ROS2 sensor_msgs](https://docs.ros2.org/latest/api/sensor_msgs/)
- [四足机器人控制理论](https://arxiv.org/abs/...)

## 许可证

Apache-2.0
