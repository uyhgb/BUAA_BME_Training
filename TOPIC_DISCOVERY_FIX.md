# ⚠️ 话题无法发现问题排查指南

## 问题症状

运行模拟器后，`ros2 topic list` 无法看到 `/lf/lowstate` 话题。

---

## 🔍 问题原因

### 根本原因：本地回环(lo)不支持多播

CycloneDDS 默认使用**多播(multicast)**进行节点发现，但是：
- ❌ 本地回环接口 `lo` **不支持多播**
- ❌ 导致不同终端的ROS2节点**无法相互发现**
- ✅ 同一终端内的节点可以通过共享内存通信

### 技术细节

```
终端1: 模拟器 (simulate_robot)
  ↓ 尝试多播通告
  ✗ 多播在 lo 上失败
  
终端2: ros2 topic list
  ↓ 尝试多播发现
  ✗ 无法发现终端1的节点
```

---

## ✅ 解决方案

### 方案1: 单终端测试（最简单）✨

在**同一个终端**中启动所有程序：

```bash
cd /home/weeq/unitree_ros2
./single_terminal_test.sh
```

这个脚本会：
1. 启动模拟器（后台）
2. 在同一终端检查话题
3. 提供交互式测试选项

**原理：** 同一终端的进程共享相同的进程空间，可以通过共享内存通信。

---

### 方案2: 使用 ROS_LOCALHOST_ONLY

修改 `setup_local.sh`，添加以下内容：

```bash
#!/bin/bash
echo "Setup unitree ros2 simulation environment"
source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 添加这两行 ⭐
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

然后在**每个终端**都source这个文件。

---

### 方案3: 使用共享内存配置（高级）

创建 CycloneDDS 配置文件，强制使用共享内存：

```bash
cat > ~/cyclonedds_shm.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <Domain id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <EnableMulticastLoopback>false</EnableMulticastLoopback>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <EnableTopicDiscoveryEndpoints>false</EnableTopicDiscoveryEndpoints>
    </Discovery>
    <Internal>
      <UseSharedMemory>true</UseSharedMemory>
    </Internal>
  </Domain>
</CycloneDDS>
EOF
```

然后设置环境变量：

```bash
export CYCLONEDDS_URI=file://$HOME/cyclonedds_shm.xml
```

---

### 方案4: 切换到 FastDDS（备选）

FastDDS 对本地回环的支持更好：

```bash
# 在 setup_local.sh 中修改
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
```

但这需要安装 FastDDS：
```bash
sudo apt install ros-humble-rmw-fastrtps-cpp
```

---

## 🧪 诊断步骤

### 步骤1: 运行诊断脚本

```bash
cd /home/weeq/unitree_ros2
./diagnose_dds.sh
```

查看输出，确认：
- ✅ RMW_IMPLEMENTATION 已设置
- ✅ CYCLONEDDS_URI 已配置
- ⚠️  注意多播警告

---

### 步骤2: 测试单终端模式

```bash
cd /home/weeq/unitree_ros2
./single_terminal_test.sh
```

如果在单终端中能看到话题，说明DDS本身工作正常，只是跨终端发现有问题。

---

### 步骤3: 手动测试

**在同一个终端中：**

```bash
cd /home/weeq/unitree_ros2/example
source ../setup_local.sh
source install/setup.bash

# 后台启动模拟器
simulate_robot &
sleep 2

# 检查话题（同一终端）
ros2 topic list
ros2 topic echo /lf/lowstate --once
```

如果能看到话题，证明问题就是多播/节点发现问题。

---

## 📊 各方案对比

| 方案 | 难度 | 跨终端 | 推荐度 |
|------|------|--------|--------|
| 方案1: 单终端 | ⭐ 简单 | ❌ 否 | ⭐⭐⭐⭐⭐ |
| 方案2: ROS_LOCALHOST_ONLY | ⭐⭐ 中等 | ✅ 是 | ⭐⭐⭐⭐ |
| 方案3: 共享内存配置 | ⭐⭐⭐ 复杂 | ✅ 是 | ⭐⭐⭐ |
| 方案4: 切换FastDDS | ⭐⭐⭐ 复杂 | ✅ 是 | ⭐⭐ |

---

## 💡 推荐流程

### 对于开发测试（现在）

使用 **方案1: 单终端测试**

```bash
/home/weeq/unitree_ros2/single_terminal_test.sh
```

优点：
- ✅ 最简单，立即可用
- ✅ 无需修改配置
- ✅ 适合快速测试

缺点：
- ❌ 不能在多个终端分别运行

---

### 对于长期使用

使用 **方案2: ROS_LOCALHOST_ONLY**

1. 修改 `setup_local.sh`:
```bash
sudo nano ~/unitree_ros2/setup_local.sh
# 添加:
# export ROS_LOCALHOST_ONLY=1
# export ROS_DOMAIN_ID=0
```

2. 每个终端都source:
```bash
source ~/unitree_ros2/setup_local.sh
```

3. 测试:
```bash
# 终端1
simulate_robot

# 终端2
ros2 topic list
```

---

## 🔧 快速修复脚本

创建改进的 `setup_local.sh`:

```bash
cat > ~/unitree_ros2/setup_local_fixed.sh << 'EOF'
#!/bin/bash
echo "Setup unitree ros2 simulation environment (Fixed for localhost)"
source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash

# 使用localhost模式
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0

# 本地回环配置
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces>
                        <AllowMulticast>false</AllowMulticast>
                        </General></Domain></CycloneDDS>'

echo "✓ ROS_LOCALHOST_ONLY = $ROS_LOCALHOST_ONLY"
echo "✓ ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "✓ RMW = $RMW_IMPLEMENTATION"
EOF

chmod +x ~/unitree_ros2/setup_local_fixed.sh
```

使用：
```bash
source ~/unitree_ros2/setup_local_fixed.sh
```

---

## ❓ 常见问题

### Q1: 为什么同一终端能看到话题？

**A:** 因为使用了共享内存传输，不依赖网络多播。

### Q2: 连接真实机器人时会有这个问题吗？

**A:** 不会！真实网卡（如eth0）支持多播，不会有这个问题。

### Q3: 能不能强制使用共享内存？

**A:** 可以，使用方案3的XML配置。

### Q4: 为什么官方示例没这个问题？

**A:** 官方示例假设连接真实机器人，使用支持多播的实际网卡。

---

## 🎯 总结

**现在就能用：**
```bash
/home/weeq/unitree_ros2/single_terminal_test.sh
```

**长期方案：**
```bash
source ~/unitree_ros2/setup_local_fixed.sh
# 然后在任何终端都能正常工作
```

**关键理解：**
- 本地回环限制：不支持多播
- 解决方法：使用localhost模式或共享内存
- 真实机器人：无此问题

---

需要帮助？运行诊断：
```bash
/home/weeq/unitree_ros2/diagnose_dds.sh
```
