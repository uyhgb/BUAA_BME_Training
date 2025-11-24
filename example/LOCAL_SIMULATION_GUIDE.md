# 🤖 无机器人模拟测试指南

## 📋 问题说明

**问题：** 使用本地回环 `lo` 作为网卡后，`ros2 topic list` 看不到机器人话题。

**原因：** 这是**正常的**！因为：
- 本地回环模式只是配置了网络接口
- **没有实际机器人 = 没有发布者 = 看不到话题**
- 您只会看到 ROS2 的默认话题：`/parameter_events` 和 `/rosout`

---

## ✅ 解决方案：使用模拟器

我已经为您创建了一个**机器人状态模拟器**，可以在没有实际机器人的情况下测试您的传感器读取程序！

---

## 🚀 快速开始（3个终端）

### 终端1: 运行模拟器（模拟机器人）

```bash
# 设置环境
cd /home/weeq/unitree_ros2/example
source ../setup_local.sh  # 使用本地回环
source install/setup.bash

# 运行模拟器
ros2 run unitree_ros2_example simulate_robot
```

**输出示例：**
```
========================================
   模拟宇树机器人状态发布器
========================================

🤖 模拟机器人状态发布器已启动
📡 发布话题: /lf/lowstate (10Hz)
💡 可以运行 custom_sensor_reader 来接收数据

📊 已发布 10 条消息 | IMU: [0.050, 0.030, 0.100] | 电池: 24.50V, 2.00A
```

### 终端2: 查看话题（验证）

```bash
# 新终端
cd /home/weeq/unitree_ros2
source setup_local.sh

# 查看话题列表
ros2 topic list
```

**现在应该看到：**
```
/lf/lowstate          ← 模拟的低层状态
/parameter_events
/rosout
```

**查看数据流：**
```bash
ros2 topic hz /lf/lowstate     # 查看频率（应该是10Hz）
ros2 topic echo /lf/lowstate   # 查看实时数据
```

### 终端3: 运行传感器读取器

```bash
# 新终端
cd /home/weeq/unitree_ros2/example
source ../setup_local.sh
source install/setup.bash

# 运行您的自定义传感器读取器
ros2 run unitree_ros2_example custom_sensor_reader
```

**输出示例：**
```
[INFO] 启动自定义传感器读取节点...
[INFO] 订阅话题: lf/lowstate
[INFO] [IMU] Roll: 0.050, Pitch: 0.030, Yaw: 0.100
[INFO] [IMU] 加速度 ax: 0.100, ay: 0.000, az: 9.810
[INFO] [电机数据]
[INFO]   电机[0] 位置: 0.500 rad, 速度: 0.500 rad/s, 扭矩: -5.000 N·m, 温度: 42°C
[INFO]   电机[1] 位置: 0.479 rad, 速度: 0.438 rad/s, 扭矩: -4.790 N·m, 温度: 43°C
...
[INFO] [电池] 电压: 24.50 V, 电流: 2.00 A, 功率: 49.00 W
```

---

## 🔍 模拟器详细信息

### 发布的数据

模拟器会发布以下**动态变化**的数据：

| 数据类型 | 模拟内容 | 变化规律 |
|---------|---------|---------|
| **IMU姿态** | Roll, Pitch, Yaw | 正弦波振荡 |
| **IMU加速度** | ax, ay, az | 正弦波 + 重力 |
| **IMU陀螺仪** | wx, wy, wz | 余弦波变化 |
| **电机位置** | 12个电机关节角 | 正弦波运动 |
| **电机速度** | 角速度 | 余弦波变化 |
| **电机扭矩** | 反馈扭矩 | 基于位置计算 |
| **电机温度** | 30-50°C | 缓慢变化 |
| **足端力** | 4个足端 | 200-300 范围 |
| **电池电压** | 24.5V → 24.0V | 缓慢下降 |
| **电池电流** | 1.5A - 2.5A | 正弦波变化 |

### 发布频率

- **话题:** `/lf/lowstate`
- **频率:** 10 Hz（每100ms一次）
- **类型:** `unitree_go/msg/LowState`

---

## 📊 工作流程图

```
┌─────────────────────┐
│  模拟器 (终端1)      │
│  simulate_robot     │
└──────────┬──────────┘
           │ 发布
           │ /lf/lowstate (10Hz)
           ▼
    ┌──────────────┐
    │  DDS 网络     │
    │  (本地回环)   │
    └──────┬───────┘
           │ 订阅
           ▼
┌─────────────────────┐
│  传感器读取器       │
│  custom_sensor_     │
│  reader (终端3)     │
└─────────────────────┘
```

---

## 🎓 学习和测试

### 测试1: 验证数据接收

```bash
# 终端1: 启动模拟器
ros2 run unitree_ros2_example simulate_robot

# 终端2: 实时查看数据
ros2 topic echo /lf/lowstate --no-arr | head -50
```

### 测试2: 检查数据频率

```bash
ros2 topic hz /lf/lowstate
# 应该显示: average rate: 10.000
```

### 测试3: 查看话题信息

```bash
ros2 topic info /lf/lowstate -v
```

### 测试4: 运行数据记录器

```bash
# 记录到CSV文件
ros2 run unitree_ros2_example custom_sensor_logger

# 几秒后按 Ctrl+C，会生成类似的文件：
# sensor_data_20251110_153045.csv
```

---

## 🔧 自定义模拟数据

如果您想修改模拟的数据，编辑 `simulate_robot.cpp`：

```cpp
// 修改IMU数据范围
msg.imu_state.rpy[0] = 0.1 * std::sin(t * 0.5);  // 加大roll幅度

// 修改电机数量或范围
for (int i = 0; i < 6; i++) {  // 只模拟6个电机
    msg.motor_state[i].q = 1.0 * std::sin(t + i);  // 加大运动范围
}

// 修改电池电压
msg.power_v = 25.0;  // 固定电压

// 修改发布频率
timer_ = this->create_wall_timer(
    50ms,  // 改为20Hz
    std::bind(&SimulatedRobotPublisher::publish_state, this));
```

重新编译：
```bash
cd /home/weeq/unitree_ros2/example
colcon build --packages-select unitree_ros2_example
```

---

## 🆚 实际机器人 vs 模拟器对比

| 特性 | 实际机器人 | 模拟器 |
|------|-----------|--------|
| **连接方式** | 网线，指定网卡 | 本地回环 `lo` |
| **话题来源** | 机器人硬件 | 模拟程序 |
| **数据真实性** | 真实传感器数据 | 数学函数生成 |
| **开发调试** | 需要硬件 | 随时可用 ✅ |
| **数据频率** | 高频500Hz/低频 | 可配置（默认10Hz）|
| **适用场景** | 实际控制 | 开发、测试、演示 |

---

## 💡 常见问题

### Q1: 为什么用本地回环看不到话题？

**A:** 因为没有发布者！
- 本地回环只是网络配置
- 需要有程序发布话题才能看到
- 解决方案：运行模拟器

### Q2: 模拟器可以用来控制真实机器人吗？

**A:** 不能！
- 模拟器只发布状态，不接收控制命令
- 它只用于测试**传感器读取程序**
- 真实控制需要连接实际机器人

### Q3: 可以同时运行多个传感器读取器吗？

**A:** 可以！
```bash
# 终端1: 模拟器
ros2 run unitree_ros2_example simulate_robot

# 终端2: 读取器1
ros2 run unitree_ros2_example custom_sensor_reader

# 终端3: 读取器2（同时运行）
ros2 run unitree_ros2_example custom_sensor_reader

# 终端4: 数据记录器（也可同时运行）
ros2 run unitree_ros2_example custom_sensor_logger
```

### Q4: 如何停止模拟器？

**A:** 在模拟器终端按 `Ctrl+C`

---

## 🎯 完整测试流程

### 步骤1: 编译（如果还没编译）

```bash
cd /home/weeq/unitree_ros2/example
source ../setup_local.sh
colcon build --packages-select unitree_ros2_example
```

### 步骤2: 启动模拟器

```bash
# 新终端
cd /home/weeq/unitree_ros2/example
source ../setup_local.sh
source install/setup.bash
ros2 run unitree_ros2_example simulate_robot
```

### 步骤3: 验证话题

```bash
# 新终端
cd /home/weeq/unitree_ros2
source setup_local.sh
ros2 topic list  # 应该看到 /lf/lowstate
```

### 步骤4: 测试读取器

```bash
# 新终端
cd /home/weeq/unitree_ros2/example
source ../setup_local.sh
source install/setup.bash
ros2 run unitree_ros2_example custom_sensor_reader
```

### 步骤5: 观察输出

您应该看到传感器读取器不断输出模拟的传感器数据！

---

## 🚀 一键启动脚本

创建测试脚本方便使用：

```bash
cat > ~/test_sensor.sh << 'EOF'
#!/bin/bash
cd /home/weeq/unitree_ros2/example
source ../setup_local.sh
source install/setup.bash

echo "请选择要运行的程序："
echo "1) 模拟器（模拟机器人）"
echo "2) 传感器读取器"
echo "3) 数据记录器"
read -p "输入选择 [1-3]: " choice

case $choice in
    1) ros2 run unitree_ros2_example simulate_robot ;;
    2) ros2 run unitree_ros2_example custom_sensor_reader ;;
    3) ros2 run unitree_ros2_example custom_sensor_logger ;;
    *) echo "无效选择" ;;
esac
EOF

chmod +x ~/test_sensor.sh
```

使用：
```bash
~/test_sensor.sh
```

---

## 📝 总结

**关键点：**

1. ✅ **本地回环看不到话题是正常的** - 因为没有机器人发布数据
2. ✅ **使用模拟器解决问题** - 模拟机器人发布状态
3. ✅ **完整测试环境** - 可以开发和测试所有传感器读取功能
4. ✅ **与真实机器人接口一致** - 代码无需修改即可用于真实机器人

**下一步：**
- 开发完成后，连接实际机器人
- 修改 `setup.sh` 中的网卡配置为实际网卡
- 运行相同的传感器读取程序即可

祝您开发顺利！🎉
