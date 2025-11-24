# 🎯 无机器人本地测试 - 快速指南

## ❓ 问题
使用本地回环 `lo` 网卡后，`ros2 topic list` 看不到机器人话题。

## ✅ 解决方案
**这是正常的！** 因为没有实际机器人就没有数据发布者。

**解决办法：使用模拟器！**

---

## 🚀 最快的使用方式

### 一键启动脚本

```bash
/home/weeq/unitree_ros2/local_test.sh
```

然后按提示选择：
1. 模拟器（模拟机器人）
2. 传感器读取器
3. 数据记录器
4-6. 各种查看工具

---

## 📝 手动使用（推荐用于学习）

### 步骤1: 启动模拟器（终端1）

```bash
cd /home/weeq/unitree_ros2/example
source ../setup_local.sh
source install/setup.bash
/home/weeq/unitree_ros2/example/install/unitree_ros2_example/bin/simulate_robot
```

### 步骤2: 验证话题（终端2）

```bash
cd /home/weeq/unitree_ros2
source setup_local.sh
ros2 topic list  # 现在应该能看到 /lf/lowstate
```

### 步骤3: 运行传感器读取器（终端3）

```bash
cd /home/weeq/unitree_ros2/example
source ../setup_local.sh
source install/setup.bash
/home/weeq/unitree_ros2/example/install/unitree_ros2_example/bin/custom_sensor_reader
```

---

## 📚 详细文档

- **完整说明:** `/home/weeq/unitree_ros2/example/LOCAL_SIMULATION_GUIDE.md`
- **自定义传感器开发:** `/home/weeq/unitree_ros2/CUSTOM_SENSOR_GUIDE.md`
- **编译错误修复:** `/home/weeq/unitree_ros2/example/COMPILE_ERROR_FIX.md`

---

## 💡 关键点

1. **本地回环看不到话题 = 正常** ← 没有发布者
2. **运行模拟器** ← 创建发布者
3. **然后就能看到话题了** ← 可以测试您的代码

---

## 🎓 工作原理

```
模拟器 → 发布 /lf/lowstate → DDS网络 → 您的程序订阅
```

模拟器就像一个"虚拟机器人"，发布与真实机器人相同格式的数据。

---

## 🔄 切换到真实机器人

开发完成后，连接真实机器人只需：

1. 用网线连接机器人
2. 修改 `setup.sh` 中的网卡为实际网卡（如 `enp3s0`）
3. `source ~/unitree_ros2/setup.sh`
4. 运行相同的程序

**代码完全不用改！** ✨

---

**祝测试顺利！** 🎉
