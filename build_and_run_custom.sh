#!/bin/bash
# 自定义传感器读取程序 - 快速编译和运行脚本

echo "================================================"
echo "  自定义传感器读取程序 - 编译和运行"
echo "================================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否在正确的目录
if [ ! -d "/home/weeq/unitree_ros2/example" ]; then
    echo -e "${RED}错误: 请确保在正确的目录运行此脚本${NC}"
    exit 1
fi

# 步骤1: 设置环境
echo -e "${YELLOW}[1/3] 设置ROS2环境...${NC}"
cd /home/weeq/unitree_ros2
source ./setup.sh
if [ $? -ne 0 ]; then
    echo -e "${RED}环境设置失败${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 环境设置完成${NC}"

# 步骤2: 编译
echo -e "${YELLOW}[2/3] 编译程序...${NC}"
cd /home/weeq/unitree_ros2/example
colcon build --packages-select unitree_ros2_example
if [ $? -ne 0 ]; then
    echo -e "${RED}编译失败！请检查错误信息${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 编译成功${NC}"

# 步骤3: 提供运行选项
echo -e "${YELLOW}[3/3] 准备运行...${NC}"
source install/setup.bash

echo ""
echo "================================================"
echo -e "${GREEN}编译完成！现在可以运行以下程序：${NC}"
echo "================================================"
echo ""
echo "1. 基础传感器读取器（实时显示传感器数据）："
echo -e "   ${YELLOW}ros2 run unitree_ros2_example custom_sensor_reader${NC}"
echo ""
echo "2. 传感器数据记录器（保存到CSV文件）："
echo -e "   ${YELLOW}ros2 run unitree_ros2_example custom_sensor_logger${NC}"
echo ""
echo "3. 原始低层状态读取："
echo -e "   ${YELLOW}ros2 run unitree_ros2_example read_low_state${NC}"
echo ""
echo "================================================"
echo ""

# 询问用户是否立即运行
read -p "是否立即运行程序？[1]基础读取器 [2]数据记录器 [n]不运行: " choice

case $choice in
    1)
        echo -e "${GREEN}启动基础传感器读取器...${NC}"
        echo "按 Ctrl+C 停止"
        sleep 1
        ros2 run unitree_ros2_example custom_sensor_reader
        ;;
    2)
        echo -e "${GREEN}启动传感器数据记录器...${NC}"
        echo "数据将保存到当前目录的CSV文件"
        echo "按 Ctrl+C 停止"
        sleep 1
        ros2 run unitree_ros2_example custom_sensor_logger
        ;;
    *)
        echo "退出。您可以稍后手动运行程序。"
        ;;
esac
