cmake_minimum_required(VERSION 3.8)
project(custom_robot_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

# Sensor to LowState Bridge Node
add_executable(sensor_bridge_node src/sensor_bridge_node.cpp)
ament_target_dependencies(sensor_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  unitree_go
)

# Custom Robot Controller Node
add_executable(custom_robot_controller src/custom_robot_controller.cpp)
ament_target_dependencies(custom_robot_controller
  rclcpp
  sensor_msgs
  unitree_go
  unitree_api
)

# Install targets
install(TARGETS
  sensor_bridge_node
  custom_robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
