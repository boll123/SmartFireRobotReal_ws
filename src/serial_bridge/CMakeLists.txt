cmake_minimum_required(VERSION 3.8)
project(serial_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(serial_bridge_node src/serial_bridge_node.cpp)
ament_target_dependencies(serial_bridge_node
  rclcpp std_msgs sensor_msgs)

install(TARGETS serial_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()