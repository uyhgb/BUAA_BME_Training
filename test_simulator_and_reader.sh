#!/bin/bash

# 测试模拟器和传感器读取器
# 用法: ./test_simulator_and_reader.sh

echo "========================================"
echo "  Unitree ROS2 本地仿真测试脚本"
echo "========================================"
echo ""

# 设置环境
source /home/weeq/unitree_ros2/setup_local_fixed.sh

echo "✓ 环境已配置"
echo ""

# 启动模拟器（后台）
echo "🚀 启动模拟器..."
ros2 run unitree_ros2_example simulate_robot > /tmp/simulator.log 2>&1 &
SIM_PID=$!
echo "✓ 模拟器 PID: $SIM_PID"
sleep 2

# 检查话题
echo ""
echo "📡 检查可用话题:"
ros2 topic list
echo ""

# 测试传感器读取器
echo "🔍 启动传感器读取器 (5秒)..."
timeout 5 ros2 run unitree_ros2_example custom_sensor_reader 2>&1 | head -20
echo ""

# 清理
echo "🛑 停止模拟器..."
kill $SIM_PID 2>/dev/null
wait $SIM_PID 2>/dev/null
echo "✓ 完成"
