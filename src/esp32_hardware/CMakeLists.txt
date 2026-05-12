cmake_minimum_required(VERSION 3.8)
project(esp32_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake      REQUIRED)
find_package(rclcpp           REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib        REQUIRED)
find_package(std_msgs         REQUIRED)
find_package(std_srvs         REQUIRED)
find_package(geometry_msgs    REQUIRED)
find_package(sensor_msgs      REQUIRED)
find_package(nav_msgs         REQUIRED)   # [NEW] สำหรับ odom_drift_corrector
find_package(tf2              REQUIRED)
find_package(tf2_ros          REQUIRED)   # [NEW] สำหรับ odom_drift_corrector
find_package(tf2_geometry_msgs REQUIRED)  # [NEW] สำหรับ odom_drift_corrector

# ── Shared library: esp32_hardware_interface ──────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/esp32_hardware_interface.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  geometry_msgs
  sensor_msgs
)
pluginlib_export_plugin_description_file(hardware_interface esp32_hardware.xml)

# ── Executable: imu_gyro_integrator ───────────────────────────────────────
add_executable(imu_gyro_integrator
  src/imu_gyro_integrator_node.cpp
)
target_include_directories(imu_gyro_integrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(imu_gyro_integrator
  rclcpp
  sensor_msgs
  tf2
  std_srvs
)

# ── Executable: odom_drift_corrector ──────────────────────────────────────
add_executable(odom_drift_corrector
  src/odom_drift_corrector_node.cpp
)
target_include_directories(odom_drift_corrector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(odom_drift_corrector
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ── Install ────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  imu_gyro_integrator
  odom_drift_corrector   # [NEW]
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES esp32_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()