#!/bin/bash
# ROS2 DDS 连接诊断脚本

echo "=========================================="
echo "  ROS2 DDS 连接诊断"
echo "=========================================="
echo ""

# 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "1. 检查环境变量"
echo "----------------------------------------"
echo "RMW_IMPLEMENTATION = ${RMW_IMPLEMENTATION:-未设置}"
echo "ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-0 (默认)}"
echo "CYCLONEDDS_URI = ${CYCLONEDDS_URI:0:100}..."
echo ""

echo "2. 检查正在运行的ROS2进程"
echo "----------------------------------------"
ps aux | grep -E "(simulate_robot|custom_sensor|ros2)" | grep -v grep
echo ""

echo "3. 测试话题发现（5秒超时）"
echo "----------------------------------------"
timeout 5 ros2 topic list 2>&1 || echo -e "${RED}超时或失败${NC}"
echo ""

echo "4. 检查网络接口"
echo "----------------------------------------"
ip addr show lo | grep "inet "
echo ""

echo "5. 可能的问题和解决方案"
echo "----------------------------------------"
echo ""

if [ -z "$RMW_IMPLEMENTATION" ]; then
    echo -e "${RED}❌ 问题: RMW_IMPLEMENTATION 未设置${NC}"
    echo "   解决: source ~/unitree_ros2/setup_local.sh"
    echo ""
fi

if [ -z "$CYCLONEDDS_URI" ]; then
    echo -e "${RED}❌ 问题: CYCLONEDDS_URI 未设置${NC}"
    echo "   解决: source ~/unitree_ros2/setup_local.sh"
    echo ""
fi

# 检查多播问题
echo -e "${YELLOW}💡 关键信息:${NC}"
echo "   本地回环(lo)不支持多播(multicast)"
echo "   这会导致话题发现变慢或失败"
echo ""
echo "   解决方案："
echo "   1) 在同一个终端中运行发布者和订阅者"
echo "   2) 或使用共享内存传输（见下方）"
echo ""

echo "6. 推荐的测试方法"
echo "----------------------------------------"
echo ""
echo "方法A: 在同一终端中测试（推荐）"
echo "   cd /home/weeq/unitree_ros2/example"
echo "   source ../setup_local.sh && source install/setup.bash"
echo "   # 启动模拟器（后台）"
echo "   simulate_robot &"
echo "   sleep 2"
echo "   # 在同一终端查看话题"
echo "   ros2 topic list"
echo ""

echo "方法B: 使用共享内存（高级）"
echo "   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
echo "   export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/config.xml"
echo ""

echo "方法C: 使用ROS_LOCALHOST_ONLY"
echo "   export ROS_LOCALHOST_ONLY=1"
echo "   export ROS_DOMAIN_ID=0"
echo ""

echo "=========================================="
echo "  诊断完成"
echo "=========================================="
