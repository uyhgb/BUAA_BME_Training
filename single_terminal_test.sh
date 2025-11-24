#!/bin/bash
# 单终端测试脚本 - 解决DDS发现问题

echo "=========================================="
echo "  单终端集成测试"
echo "=========================================="
echo ""

cd /home/weeq/unitree_ros2/example

# 设置环境
echo "1. 设置环境..."
source ../setup_local.sh
source install/setup.bash

# 确保没有旧进程
pkill -f simulate_robot 2>/dev/null
sleep 1

echo ""
echo "2. 启动模拟器（后台）..."
/home/weeq/unitree_ros2/example/install/unitree_ros2_example/bin/simulate_robot > /tmp/simulator.log 2>&1 &
SIM_PID=$!

echo "   模拟器 PID: $SIM_PID"
echo "   等待启动..."
sleep 3

echo ""
echo "3. 检查话题列表..."
ros2 topic list

echo ""
echo "4. 查看话题详情..."
timeout 3 ros2 topic info /lf/lowstate 2>/dev/null || echo "话题信息获取失败"

echo ""
echo "5. 监听话题数据（5秒）..."
timeout 5 ros2 topic echo /lf/lowstate --once 2>/dev/null || echo "未收到数据"

echo ""
echo "=========================================="
echo "  测试选项"
echo "=========================================="
echo ""
echo "选择操作："
echo "1) 运行传感器读取器"
echo "2) 持续监听话题"
echo "3) 查看模拟器日志"
echo "4) 停止模拟器"
echo "5) 退出"
echo ""
read -p "输入选择 [1-5]: " choice

case $choice in
    1)
        echo ""
        echo "启动传感器读取器..."
        /home/weeq/unitree_ros2/example/install/unitree_ros2_example/bin/custom_sensor_reader
        ;;
    2)
        echo ""
        echo "持续监听话题（按Ctrl+C停止）..."
        ros2 topic echo /lf/lowstate
        ;;
    3)
        echo ""
        echo "模拟器日志:"
        cat /tmp/simulator.log
        ;;
    4)
        echo ""
        echo "停止模拟器..."
        kill $SIM_PID 2>/dev/null
        echo "已停止"
        ;;
    5)
        echo ""
        echo "退出（模拟器仍在后台运行）"
        echo "如需停止: kill $SIM_PID"
        ;;
esac

echo ""
echo "测试完成"
