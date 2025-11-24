#!/bin/bash
echo "Setup unitree ros2 environment with default interface"
source /opt/ros/humble/setup.bash
source $HOME/unitree_ros2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
