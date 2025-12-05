cmake_minimum_required(VERSION 3.8)
project(imu_serial_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add nlohmann/json (header-only library)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../include)

# IMU Serial Publisher Node
add_executable(imu_serial_publisher_node src/imu_serial_publisher.cpp)
ament_target_dependencies(imu_serial_publisher_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# Install targets
install(TARGETS
  imu_serial_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
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
