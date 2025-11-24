#!/bin/bash
echo "Setup unitree ros2 simulation environment (with localhost discovery fix)"
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash

# example 目录的 install 是旧的，跳过
# source /workspace/example/install/setup.bash

# ROS2 DDS implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 关键修复：启用localhost模式，解决本地回环多播问题
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0

# CycloneDDS配置 - 使用简化配置，避免接口重复
unset CYCLONEDDS_URI

echo "✓ ROS_LOCALHOST_ONLY = $ROS_LOCALHOST_ONLY (enabled for local testing)"
echo "✓ ROS_DOMAIN_ID = $ROS_DOMAIN_ID"
echo "✓ RMW_IMPLEMENTATION = $RMW_IMPLEMENTATION"
echo "✓ CYCLONEDDS_URI = (unset, using default with ROS_LOCALHOST_ONLY)"
echo ""
echo "💡 Tip: This configuration allows topic discovery across different terminals"
