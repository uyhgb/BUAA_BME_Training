# 🚀 快速参考卡片

## 📌 已创建的文件

### 1. 示例代码
- **`/home/weeq/unitree_ros2/example/src/src/custom_sensor_reader.cpp`**
  - 基础传感器读取器
  - 可配置读取IMU、电机、电池等数据
  - 预留自定义处理接口

- **`/home/weeq/unitree_ros2/example/src/src/custom_sensor_logger.cpp`**
  - 传感器数据记录器
  - 自动保存CSV文件
  - 可发布到自定义话题

### 2. 文档
- **`/home/weeq/unitree_ros2/CUSTOM_SENSOR_GUIDE.md`**
  - 完整开发指南
  - 使用说明和示例

### 3. 脚本
- **`/home/weeq/unitree_ros2/build_and_run_custom.sh`**
  - 一键编译和运行脚本

### 4. 配置
- **`/home/weeq/unitree_ros2/example/src/CMakeLists.txt`**
  - 已添加编译配置

---

## ⚡ 快速开始（3步）

### 方法1: 使用自动化脚本
```bash
cd /home/weeq/unitree_ros2
./build_and_run_custom.sh
```

### 方法2: 手动编译运行
```bash
# 步骤1: 设置环境
cd /home/weeq/unitree_ros2
source ./setup.sh

# 步骤2: 编译
cd example
colcon build

# 步骤3: 运行
source install/setup.bash
ros2 run unitree_ros2_example custom_sensor_reader
```

---

## 🎯 常用命令

### 查看所有话题
```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
```

### 实时监控低层状态
```bash
ros2 topic echo /lf/lowstate
```

### 检查话题频率
```bash
ros2 topic hz /lf/lowstate
```

### 查看话题信息
```bash
ros2 topic info /lf/lowstate -v
```

---

## 🔧 自定义配置

### 修改读取的传感器（custom_sensor_reader.cpp）
```cpp
// 文件顶部配置
constexpr bool READ_IMU = true;         // IMU
constexpr bool READ_MOTOR = true;       // 电机
constexpr bool READ_FOOT_FORCE = false; // 足端力
constexpr bool READ_BATTERY = true;     // 电池
constexpr bool USE_HIGH_FREQ = false;   // 高频模式

// 选择特定电机
const std::vector<int> SELECTED_MOTORS = {0, 1, 2, 3};
```

### 修改数据采样率（custom_sensor_logger.cpp）
```cpp
// 在sensor_callback函数中
if (sample_count_ % 10 != 0) {  // 改为您需要的采样间隔
    return;
}
```

---

## 📊 可用的传感器数据

| 传感器类型 | 数据内容 | 单位 |
|-----------|---------|------|
| **IMU** | 欧拉角 (roll, pitch, yaw) | rad |
| | 四元数 (w, x, y, z) | - |
| | 陀螺仪 (wx, wy, wz) | rad/s |
| | 加速度 (ax, ay, az) | m/s² |
| **电机** | 位置 (q) | rad |
| | 速度 (dq) | rad/s |
| | 加速度 (ddq) | rad/s² |
| | 扭矩 (tau_est) | N·m |
| | 温度 | °C |
| **电池** | 电压 | V |
| | 电流 | A |
| **足端力** | 实测值 / 估计值 | - |

---

## 📍 主要话题

| 话题名称 | 类型 | 频率 | 说明 |
|---------|------|------|------|
| `/lowstate` | LowState | 500Hz | 低层状态（高频） |
| `/lf/lowstate` | LowState | 低频 | 低层状态（低频） |
| `/sportmodestate` | SportModeState | - | 运动模式状态 |
| `/wirelesscontroller` | WirelessController | - | 无线控制器 |

---

## 💡 代码模板

### 订阅自定义传感器话题
```cpp
// 在您的ROS2节点中
auto sub = this->create_subscription<sensor_msgs::msg::Imu>(
    "my_sensor_topic", 10,
    [this](sensor_msgs::msg::Imu::SharedPtr msg) {
        // 处理传感器数据
        RCLCPP_INFO(this->get_logger(), "收到数据");
    });
```

### 发布处理后的数据
```cpp
// 创建发布者
auto pub = this->create_publisher<std_msgs::msg::String>(
    "processed_data", 10);

// 发布数据
auto msg = std_msgs::msg::String();
msg.data = "处理后的数据";
pub->publish(msg);
```

---

## ⚠️ 常见问题

### 问题1: 看不到话题
**解决：** 确保已正确设置环境
```bash
source ~/unitree_ros2/setup.sh
```

### 问题2: 编译失败
**解决：** 检查依赖是否安装
```bash
sudo apt install ros-humble-rclcpp ros-humble-std-msgs
```

### 问题3: 没有数据
**解决：** 检查机器人是否连接，网络接口是否正确配置

---

## 📚 下一步

1. ✅ 阅读完整指南: `CUSTOM_SENSOR_GUIDE.md`
2. ✅ 修改示例代码适配您的需求
3. ✅ 测试并验证数据正确性
4. ✅ 整合到您的机器人系统中

---

## 🆘 获取帮助

- **宇树官方文档**: https://support.unitree.com
- **ROS2文档**: https://docs.ros.org
- **GitHub Issues**: 提问和反馈

---

**祝您开发顺利！** 🎉
