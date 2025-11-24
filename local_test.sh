#!/bin/bash
# 本地模拟测试 - 快速启动脚本

echo "================================================"
echo "  宇树ROS2 本地模拟测试"
echo "================================================"
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

cd /home/weeq/unitree_ros2/example

# 检查是否已编译
if [ ! -f "install/unitree_ros2_example/bin/simulate_robot" ]; then
    echo -e "${YELLOW}⚠️  检测到尚未编译，正在编译...${NC}"
    source ../setup_local.sh
    colcon build --packages-select unitree_ros2_example
    if [ $? -ne 0 ]; then
        echo -e "${RED}编译失败！${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ 编译完成${NC}"
    echo ""
fi

# 设置环境
source ../setup_local.sh
source install/setup.bash

echo "请选择要运行的程序："
echo ""
echo -e "${BLUE}1)${NC} 🤖 模拟器 (simulate_robot)"
echo "   - 模拟机器人发布状态数据"
echo "   - 发布话题: /lf/lowstate"
echo "   - 频率: 10Hz"
echo ""
echo -e "${BLUE}2)${NC} 📡 传感器读取器 (custom_sensor_reader)"
echo "   - 订阅并显示传感器数据"
echo "   - 需要先运行模拟器"
echo ""
echo -e "${BLUE}3)${NC} 💾 数据记录器 (custom_sensor_logger)"
echo "   - 记录数据到CSV文件"
echo "   - 需要先运行模拟器"
echo ""
echo -e "${BLUE}4)${NC} 🔍 查看话题列表"
echo ""
echo -e "${BLUE}5)${NC} 📊 监控话题数据 (echo)"
echo ""
echo -e "${BLUE}6)${NC} ⚡ 查看话题频率 (hz)"
echo ""

read -p "输入选择 [1-6]: " choice

echo ""
echo "================================================"

case $choice in
    1)
        echo -e "${GREEN}启动模拟器...${NC}"
        echo "按 Ctrl+C 停止"
        echo ""
        exec /home/weeq/unitree_ros2/example/install/unitree_ros2_example/bin/simulate_robot
        ;;
    2)
        echo -e "${GREEN}启动传感器读取器...${NC}"
        echo "确保模拟器已在另一个终端运行！"
        echo "按 Ctrl+C 停止"
        echo ""
        exec /home/weeq/unitree_ros2/example/install/unitree_ros2_example/bin/custom_sensor_reader
        ;;
    3)
        echo -e "${GREEN}启动数据记录器...${NC}"
        echo "确保模拟器已在另一个终端运行！"
        echo "数据将保存到 CSV 文件"
        echo "按 Ctrl+C 停止"
        echo ""
        exec /home/weeq/unitree_ros2/example/install/unitree_ros2_example/bin/custom_sensor_logger
        ;;
    4)
        echo -e "${GREEN}话题列表:${NC}"
        echo ""
        ros2 topic list
        echo ""
        echo "💡 提示: 如果只看到 /parameter_events 和 /rosout"
        echo "   说明模拟器还没运行，在另一个终端运行选项1"
        ;;
    5)
        echo -e "${GREEN}实时监控话题数据...${NC}"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 topic echo /lf/lowstate
        ;;
    6)
        echo -e "${GREEN}检查话题频率...${NC}"
        echo "按 Ctrl+C 停止"
        echo ""
        ros2 topic hz /lf/lowstate
        ;;
    *)
        echo -e "${YELLOW}无效选择${NC}"
        exit 1
        ;;
esac
