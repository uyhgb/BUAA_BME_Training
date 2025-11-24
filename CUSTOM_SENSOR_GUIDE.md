# 自定义传感器读取指南

本指南说明如何基于宇树ROS2功能包开发自己的传感器读取程序。

## 📋 概述

您**完全可以**基于这个仓库的代码实现自己的传感器读取方案。主要有以下几种方式：

### 方式1: 直接订阅现有话题 ✅ **推荐**
- **不需要修改**宇树官方代码
- 创建自己的ROS2节点订阅传感器话题
- 最灵活、最安全的方式

### 方式2: 修改示例代码
- 可以修改 `example/src/src/` 中的示例代码
- 适合学习和快速原型开发

### 方式3: 创建独立功能包
- 创建完全独立的ROS2功能包
- 最专业的做法，便于维护和分发

---

## 🚀 快速开始 - 方式1（推荐）

我已经为您创建了两个示例代码：

### 1. 基础传感器读取器 `custom_sensor_reader.cpp`
**功能：** 订阅机器人低层状态话题，提取所需传感器数据

**特点：**
- ✅ 可选择读取IMU、电机、足端力、电池等传感器
- ✅ 可配置高频/低频模式
- ✅ 可选择特定电机读取
- ✅ 预留了自定义数据处理接口

**使用场景：**
- 实时监控机器人状态
- 获取特定传感器数据
- 作为上位机传感器读取程序

### 2. 传感器数据记录器 `custom_sensor_logger.cpp`
**功能：** 记录传感器数据到CSV文件，并转发到自定义话题

**特点：**
- ✅ 自动生成带时间戳的CSV日志文件
- ✅ 可发布到自定义ROS2话题
- ✅ 支持数据采样频率控制

**使用场景：**
- 传感器数据记录和分析
- 数据转发到其他系统
- 离线数据分析

---

## 📝 编译和运行

### 步骤1: 修改CMakeLists.txt

编辑 `/home/weeq/unitree_ros2/example/src/CMakeLists.txt`，添加以下内容：

```cmake
# 在 add_executable 部分添加
add_executable(custom_sensor_reader src/custom_sensor_reader.cpp)
add_executable(custom_sensor_logger src/custom_sensor_logger.cpp)

# 在 ament_target_dependencies 部分添加
ament_target_dependencies(custom_sensor_reader ${DEPENDENCY_LIST})
ament_target_dependencies(custom_sensor_logger ${DEPENDENCY_LIST})

# 在 install 部分添加
install(TARGETS
  # ... 其他目标 ...
  custom_sensor_reader
  custom_sensor_logger
  DESTINATION lib/${PROJECT_NAME}
)
```

### 步骤2: 编译

```bash
cd ~/unitree_ros2/example
source ~/unitree_ros2/setup.sh
colcon build
```

### 步骤3: 运行

**运行基础传感器读取器：**
```bash
source ~/unitree_ros2/example/install/setup.bash
ros2 run unitree_ros2_example custom_sensor_reader
```

**运行传感器数据记录器：**
```bash
source ~/unitree_ros2/example/install/setup.bash
ros2 run unitree_ros2_example custom_sensor_logger
```

---

## 🔧 自定义开发指南

### 1. 修改读取的传感器类型

编辑 `custom_sensor_reader.cpp` 顶部的配置：

```cpp
constexpr bool READ_IMU = true;         // 是否读取IMU
constexpr bool READ_MOTOR = true;       // 是否读取电机
constexpr bool READ_FOOT_FORCE = false; // 是否读取足端力
constexpr bool READ_BATTERY = true;     // 是否读取电池
```

### 2. 选择特定电机

```cpp
// 只读取指定ID的电机
const std::vector<int> SELECTED_MOTORS = {0, 1, 2, 3};
```

### 3. 添加自己的数据处理逻辑

在各个 `process_xxx_data()` 函数中添加您的代码：

```cpp
void process_imu_data(const unitree_go::msg::IMUState& imu) {
    // ... 现有代码 ...
    
    // ===== 在这里添加您自己的处理逻辑 =====
    // 例如：保存到数据库
    // 例如：发送到其他网络节点
    // 例如：触发控制逻辑
}
```

### 4. 订阅不同的话题

根据需要修改订阅的话题：

| 话题名称 | 频率 | 内容 |
|---------|------|------|
| `lowstate` | 500Hz | 低层状态（高频） |
| `lf/lowstate` | 低频 | 低层状态（低频） |
| `sportmodestate` | - | 运动模式状态 |
| `/wirelesscontroller` | - | 无线控制器 |

```cpp
// 修改话题名称
std::string topic_name = "lf/lowstate";  // 改为您需要的话题
```

---

## 📊 可用的传感器数据

### IMU传感器 (unitree_go::msg::IMUState)
```cpp
imu.rpy[3]            // 欧拉角: [roll, pitch, yaw]
imu.quaternion[4]     // 四元数: [w, x, y, z]
imu.gyroscope[3]      // 陀螺仪: [wx, wy, wz] (rad/s)
imu.accelerometer[3]  // 加速度: [ax, ay, az] (m/s²)
```

### 电机状态 (unitree_go::msg::MotorState)
```cpp
motor.q           // 关节角度 (rad)
motor.dq          // 角速度 (rad/s)
motor.ddq         // 角加速度 (rad/s²)
motor.tau_est     // 估计扭矩 (N·m)
motor.temperature // 温度 (°C)
motor.mode        // 工作模式
```

### 电池信息
```cpp
msg->power_v  // 电压 (V)
msg->power_a  // 电流 (A)
```

### 足端力传感器
```cpp
msg->foot_force[4]      // 实测足端力
msg->foot_force_est[4]  // 估计足端力
```

---

## 🎯 实际应用示例

### 示例1: 监控电池电量并报警

```cpp
void process_battery_data(float voltage, float current) {
    static bool low_battery_warned = false;
    
    if (voltage < 21.0 && !low_battery_warned) {
        RCLCPP_WARN(this->get_logger(), "🔋 警告：电池电量低！");
        // 发送邮件、声音提示等
        low_battery_warned = true;
    }
}
```

### 示例2: 记录电机运动轨迹

```cpp
void process_motor_data(const unitree_go::msg::MotorState motor_state[20]) {
    // 保存到文件用于后续分析
    for (int i = 0; i < 12; i++) {
        trajectory_log_ << motor_state[i].q << ",";
    }
    trajectory_log_ << "\n";
}
```

### 示例3: 实时计算机器人姿态

```cpp
void process_imu_data(const unitree_go::msg::IMUState& imu) {
    double roll = imu.rpy[0];
    double pitch = imu.rpy[1];
    
    if (std::abs(pitch) > 0.5) {  // 俯仰角超过30度
        RCLCPP_WARN(this->get_logger(), "⚠️  机器人倾斜过大！");
    }
}
```

---

## 🌐 与其他传感器通信

### 如果您有自己的传感器硬件：

#### 方案A: 传感器发布ROS2话题
1. 编写传感器驱动程序（Python或C++）
2. 发布到自定义ROS2话题
3. 在上位机订阅该话题

```cpp
// 订阅您自己的传感器话题
auto my_sensor_sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "my_custom_sensor", 10, 
    [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        // 处理您的传感器数据
    });
```

#### 方案B: 直接串口/网络通信
1. 使用C++串口库（如 `serial`）
2. 在ROS2节点中读取传感器数据
3. 可选：发布到ROS2话题供其他节点使用

```cpp
// 伪代码示例
class MySensorReader : public rclcpp::Node {
    Serial serial_port_;
    
    void read_sensor() {
        auto data = serial_port_.read();
        // 解析并发布
    }
};
```

---

## 🔍 调试技巧

### 1. 查看所有可用话题
```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
```

### 2. 查看话题数据格式
```bash
ros2 topic info /lowstate -v
```

### 3. 实时监控话题数据
```bash
ros2 topic echo /lowstate
```

### 4. 检查话题发布频率
```bash
ros2 topic hz /lowstate
```

---

## ⚠️ 注意事项

1. **不要直接修改宇树官方代码**
   - 升级时会丢失修改
   - 建议创建独立的订阅节点

2. **注意话题频率**
   - 高频话题(500Hz)会消耗较多CPU
   - 根据需要选择合适频率

3. **数据同步问题**
   - 不同传感器数据可能不完全同步
   - 需要时添加时间戳处理

4. **网络配置**
   - 确保正确配置DDS网络接口
   - 参考 `setup.sh` 配置

---

## 📚 相关资源

- **宇树官方文档**: https://support.unitree.com/home/en/developer
- **ROS2教程**: https://docs.ros.org/en/humble/Tutorials.html
- **示例代码位置**: `/home/weeq/unitree_ros2/example/src/src/`

---

## ✅ 总结

**您完全可以：**
- ✅ 订阅宇树机器人的传感器话题
- ✅ 编写自己的数据处理程序
- ✅ 保存、转发、分析传感器数据
- ✅ 整合自己的传感器到系统中

**推荐流程：**
1. 使用提供的示例代码作为起点
2. 根据需求修改传感器读取逻辑
3. 测试并验证数据正确性
4. 根据需要扩展功能

有任何问题随时询问！
