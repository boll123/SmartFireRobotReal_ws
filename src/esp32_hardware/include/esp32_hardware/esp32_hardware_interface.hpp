// esp32_hardware_interface.hpp  —  v10-MapFix
// [เพิ่ม] stop_threshold_ member (configurable via YAML)
#pragma once
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <atomic>
namespace esp32_hardware
{
class Esp32HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Esp32HardwareInterface)
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
private:
  void motorStateCb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void imuRawCb(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  double rpm_L_{0.0}, rpm_R_{0.0};
  double rpm_L_filt_{0.0}, rpm_R_filt_{0.0};
  std::mutex rpm_mutex_;
  sensor_msgs::msg::Imu imu_msg_;
  bool imu_received_{false};
  std::mutex imu_mutex_;
  // Robot params
  double wheel_radius_{0.0826};
  double wheel_separation_{0.572};
  double rpm_deadzone_{3.0};
  double rpm_lpf_alpha_{0.30};
  // Write params
  double max_omega_{7.33};
  double dead_band_{0.05};
  double trim_ratio_{1.04};
  double cmd_timeout_ms_{200.0};
  double cmd_publish_rate_{20.0};
  // [v10] stop_threshold: snap stop ที่ค่านี้แทน hardcode 0.15
  // default = max(dead_band_, 0.35) ตั้งใน on_init()
  // ปรับได้ใน YAML: stop_threshold: 0.35
  double stop_threshold_{0.35};
  // cmd sign tracking
  double last_cmd_sign_L_{1.0};
  double last_cmd_sign_R_{1.0};
  // Watchdog
  rclcpp::Time last_nonzero_cmd_time_{0, 0, RCL_ROS_TIME};
  bool timeout_stop_sent_{false};
  // Write throttle
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  float last_omega_L_{0.0f};
  float last_omega_R_{0.0f};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr imu_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    cmd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr               imu_pub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};
}  // namespace esp32_hardware