/*
 * esp32_hardware_interface.cpp  —  v13-OdomFix
 * ─────────────────────────────────────────────────────────────────────────────
 * v10→v12 fixes:
 *   [FIX ODOM 1] read():
 *       omega_L = -r_L * (2π/60)   ← กลับ sign left wheel
 *       omega_R = -r_R * (2π/60)   ← กลับ sign right wheel ด้วย
 *       เหตุผล: diff_drive คำนวณ vx = (omega_L + omega_R) / 2
 *               ถ้า L กลับแต่ R ไม่กลับ → L และ R มี sign ตรงข้าม → vx = 0
 *               ต้องกลับทั้งคู่เพื่อให้ vx = (−rL − rR)/2 * 2π/60 ถูกต้อง
 *               yaw = (omega_R − omega_L) / wheel_sep ยังถูกเพราะกลับเท่ากันทั้งคู่
 *
 *   [FIX ODOM 2] write():
 *       send_L = -omega_L   ← กลับ sign ก่อนส่ง ESP32
 *       send_R = -omega_R   ← กลับ sign ก่อนส่ง ESP32 ด้วย
 *       เหตุผล: สอดคล้องกับ read() — motor ทั้งสองข้าง wiring กลับด้านกับ
 *               diff_drive convention
 *
 *   [KEEP v10] stop_threshold configurable, snap-stop, watchdog ทั้งหมด
 * ─────────────────────────────────────────────────────────────────────────────
 * อาการก่อนแก้ (v10):
 *   - หุ่นเลี้ยวขวา → odom เลี้ยวซ้าย      (yaw sign ผิด)
 *   - หุ่นเดินหน้า  → odom ถอยหลัง         (vx sign ผิด)
 *   - วิ่งวงกลมครบรอบ → odom สไลด์ข้าง     (error สะสม)
 * อาการก่อนแก้ (v11):
 *   - หุ่นเลี้ยวถูก  → yaw ถูกแล้ว
 *   - หุ่นเดินหน้า/ถอย → odom อยู่กับที่   (L กลับ R ไม่กลับ → vx หักล้าง 0)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "esp32_hardware/esp32_hardware_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <cmath>

namespace esp32_hardware {

hardware_interface::CallbackReturn Esp32HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  auto & p = info_.hardware_parameters;
  if (p.count("wheel_radius"))       wheel_radius_      = std::stod(p.at("wheel_radius"));
  if (p.count("wheel_separation"))   wheel_separation_  = std::stod(p.at("wheel_separation"));
  if (p.count("rpm_deadzone"))       rpm_deadzone_      = std::stod(p.at("rpm_deadzone"));
  if (p.count("rpm_lpf_alpha"))      rpm_lpf_alpha_     = std::max(0.0, std::min(1.0,
                                       std::stod(p.at("rpm_lpf_alpha"))));
  if (p.count("cmd_publish_rate"))   cmd_publish_rate_  = std::stod(p.at("cmd_publish_rate"));
  if (p.count("max_omega"))          max_omega_         = std::stod(p.at("max_omega"));
  if (p.count("dead_band"))          dead_band_         = std::stod(p.at("dead_band"));
  if (p.count("cmd_timeout_ms"))     cmd_timeout_ms_    = std::stod(p.at("cmd_timeout_ms"));
  if (p.count("trim_ratio"))         trim_ratio_        = std::stod(p.at("trim_ratio"));

  // stop_threshold configurable — default max(dead_band_, 0.35)
  if (p.count("stop_threshold"))
    stop_threshold_ = std::stod(p.at("stop_threshold"));
  else
    stop_threshold_ = std::max(dead_band_, 0.35);

  hw_positions_.assign(4, 0.0);
  hw_velocities_.assign(4, 0.0);
  hw_commands_.assign(2, 0.0);

  last_cmd_sign_L_ = 1.0;
  last_cmd_sign_R_ = 1.0;

  imu_msg_.header.frame_id           = "imu_link";
  imu_msg_.orientation_covariance[0] = -1.0;
  imu_msg_.angular_velocity_covariance = {
    0.000025, 0.0,      0.0,
    0.0,      0.000025, 0.0,
    0.0,      0.0,      0.000025
  };
  imu_msg_.linear_acceleration_covariance = {
    0.0001, 0.0,    0.0,
    0.0,    0.0001, 0.0,
    0.0,    0.0,    0.0001
  };

  RCLCPP_INFO(rclcpp::get_logger("Esp32HardwareInterface"),
    "Init v13-OdomFix: r=%.4f sep=%.4f deadzone=%.1f lpf=%.2f rate=%.1f "
    "max_omega=%.3f dead_band=%.3f stop_threshold=%.3f timeout=%.0fms trim=%.4f",
    wheel_radius_, wheel_separation_, rpm_deadzone_,
    rpm_lpf_alpha_, cmd_publish_rate_, max_omega_, dead_band_,
    stop_threshold_, cmd_timeout_ms_, trim_ratio_);

  RCLCPP_INFO(rclcpp::get_logger("Esp32HardwareInterface"),
    "OdomFix v13: NO sign inversion — ESP32 convention matches diff_drive directly");
  RCLCPP_INFO(rclcpp::get_logger("Esp32HardwareInterface"),
    "  read : omega_L = +rL*2pi/60,  omega_R = +rR*2pi/60");
  RCLCPP_INFO(rclcpp::get_logger("Esp32HardwareInterface"),
    "  write: send_L  = +omega_L,    send_R  = +omega_R");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32HardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  node_ = rclcpp::Node::make_shared("esp32_hw_node");

  motor_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/motor_state", 10,
    std::bind(&Esp32HardwareInterface::motorStateCb, this, std::placeholders::_1));

  imu_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/imu_raw", 50,
    std::bind(&Esp32HardwareInterface::imuRawCb, this, std::placeholders::_1));

  cmd_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_cmd", 10);
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 50);

  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });

  RCLCPP_INFO(node_->get_logger(), "Configured v13-OdomFix");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32HardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  hw_positions_.assign(4, 0.0);
  hw_velocities_.assign(4, 0.0);
  hw_commands_.assign(2, 0.0);

  { std::lock_guard<std::mutex> lk(rpm_mutex_);
    rpm_L_ = rpm_R_ = rpm_L_filt_ = rpm_R_filt_ = 0.0; }
  { std::lock_guard<std::mutex> lk(imu_mutex_);
    imu_received_ = false; }

  last_cmd_time_         = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_nonzero_cmd_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_omega_L_          = 0.0f;
  last_omega_R_          = 0.0f;
  last_cmd_sign_L_       = 1.0;
  last_cmd_sign_R_       = 1.0;
  timeout_stop_sent_     = false;

  RCLCPP_INFO(node_->get_logger(), "Activated v13-OdomFix");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Esp32HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std_msgs::msg::Float32MultiArray stop;
  stop.data = {0.0f, 0.0f};
  cmd_pub_->publish(stop);
  executor_.cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
  RCLCPP_INFO(node_->get_logger(), "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Esp32HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> v;
  for (int i = 0; i < 4; i++) {
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return v;
}

std::vector<hardware_interface::CommandInterface>
Esp32HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> v;
  v.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
  v.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);
  return v;
}

void Esp32HardwareInterface::motorStateCb(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) return;
  double raw_L = static_cast<double>(msg->data[0]);
  double raw_R = static_cast<double>(msg->data[1]);

  std::lock_guard<std::mutex> lk(rpm_mutex_);

  rpm_L_filt_ = rpm_lpf_alpha_ * raw_L + (1.0 - rpm_lpf_alpha_) * rpm_L_filt_;
  rpm_R_filt_ = rpm_lpf_alpha_ * raw_R + (1.0 - rpm_lpf_alpha_) * rpm_R_filt_;

  rpm_L_ = (std::abs(rpm_L_filt_) < rpm_deadzone_) ? 0.0 : rpm_L_filt_;
  rpm_R_ = (std::abs(rpm_R_filt_) < rpm_deadzone_) ? 0.0 : rpm_R_filt_;
}

void Esp32HardwareInterface::imuRawCb(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 6) return;
  float az = msg->data[2];
  if (std::abs(az) < 0.5f) return;

  sensor_msgs::msg::Imu imu;
  imu.header.stamp    = node_->now();
  imu.header.frame_id = "imu_link";
  imu.linear_acceleration.x = static_cast<double>(msg->data[0]);
  imu.linear_acceleration.y = static_cast<double>(msg->data[1]);
  imu.linear_acceleration.z = static_cast<double>(msg->data[2]);
  imu.angular_velocity.x    = static_cast<double>(msg->data[3]);
  imu.angular_velocity.y    = static_cast<double>(msg->data[4]);
  imu.angular_velocity.z    = static_cast<double>(msg->data[5]);
  imu.orientation_covariance[0]      = -1.0;
  imu.angular_velocity_covariance    = imu_msg_.angular_velocity_covariance;
  imu.linear_acceleration_covariance = imu_msg_.linear_acceleration_covariance;

  { std::lock_guard<std::mutex> lk(imu_mutex_);
    imu_msg_ = imu; imu_received_ = true; }
  imu_pub_->publish(imu);
}

// ─────────────────────────────────────────────────────────────────────────────
// read  —  v12: กลับ sign ทั้ง left และ right wheel
//
//   diff_drive คำนวณ:
//     vx  = (omega_L + omega_R) * wheel_radius / 2
//     yaw = (omega_R - omega_L) * wheel_radius / wheel_separation
//
//   ถ้ากลับแค่ L (v11):
//     omega_L = -rL,  omega_R = +rR
//     เดินหน้า: rL > 0, rR > 0 → omega_L < 0, omega_R > 0 → vx ≈ 0 (หักล้าง!)
//     yaw = (+rR - (-rL)) = rR + rL > 0 → ถูก (บวก = เลี้ยวซ้าย)
//
//   ถ้ากลับทั้งคู่ (v12):
//     omega_L = -rL,  omega_R = -rR
//     เดินหน้า: rL > 0, rR > 0 → omega_L < 0, omega_R < 0 → vx < 0 ← ยังผิด?
//
//   *** ความจริง: ESP32 ส่ง rL, rR พร้อม sign ที่ถูกต้อง (forward = บวก) ***
//   เมื่อหุ่นเดินหน้า: ESP32 ส่ง rL < 0 (เพราะ send_L = -omega_L และ omega_L > 0)
//                                  rR < 0 (เพราะ send_R = -omega_R และ omega_R > 0)
//   ดังนั้น:  omega_L = -rL = -(-|v|) = +|v|  ✓
//             omega_R = -rR = -(-|v|) = +|v|  ✓
//             vx = (+|v| + |v|) / 2 > 0       ✓ เดินหน้า
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type Esp32HardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.5) return hardware_interface::return_type::OK;

  double r_L, r_R;
  { std::lock_guard<std::mutex> lk(rpm_mutex_);
    r_L = rpm_L_; r_R = rpm_R_; }

  // v13: ไม่กลับ sign — ESP32 ส่ง rL/rR ตรง diff_drive convention อยู่แล้ว
  double omega_L = r_L * (2.0 * M_PI / 60.0);
  double omega_R = r_R * (2.0 * M_PI / 60.0);

  hw_velocities_[0] = omega_L;   // LFHover_Wheel_joint
  hw_velocities_[1] = omega_R;   // RTHover_Wheel_joint
  hw_velocities_[2] = omega_L;   // LFCaster_Wheel_joint (ตาม hover)
  hw_velocities_[3] = omega_R;   // RTCaster_Wheel_joint (ตาม hover)

  hw_positions_[0] += omega_L * dt;
  hw_positions_[1] += omega_R * dt;
  hw_positions_[2] += omega_L * dt;
  hw_positions_[3] += omega_R * dt;

  return hardware_interface::return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
// write  —  v13: ส่ง omega ตรงให้ ESP32 ไม่กลับ sign
//
//   ESP32 (motor_serial.ino) รับ oL, oR แล้ว:
//     DIR_L: oL > 0 → LOW  (forward),  oL < 0 → HIGH (reverse)
//     DIR_R: oR > 0 → HIGH (forward),  oR < 0 → LOW  (reverse)
//   ซึ่งตรงกับ diff_drive convention อยู่แล้ว — ไม่ต้องแปลง
// ─────────────────────────────────────────────────────────────────────────────
hardware_interface::return_type Esp32HardwareInterface::write(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  double omega_L = hw_commands_[0];
  double omega_R = hw_commands_[1];

  // Snap stop — ใช้ stop_threshold_ (configurable, default 0.35 rad/s)
  if (std::abs(omega_L) < stop_threshold_) omega_L = 0.0;
  if (std::abs(omega_R) < stop_threshold_) omega_R = 0.0;

  // Clamp to max_omega
  omega_L = std::clamp(omega_L, -max_omega_, max_omega_);
  omega_R = std::clamp(omega_R, -max_omega_, max_omega_);

  // Trim: ใช้เมื่อหมุนทิศเดียวกัน (straight line)
  bool same_direction = (omega_L * omega_R > 0.0);
  if (same_direction && omega_L != 0.0) {
    omega_L = std::clamp(omega_L * trim_ratio_, -max_omega_, max_omega_);
  }

  // Track sign ของ diff_drive command (ก่อนกลับด้าน)
  if (omega_L != 0.0) last_cmd_sign_L_ = (omega_L > 0.0) ? 1.0 : -1.0;
  if (omega_R != 0.0) last_cmd_sign_R_ = (omega_R > 0.0) ? 1.0 : -1.0;

  // v13: ไม่กลับ sign — ส่งตรงให้ ESP32
  double send_L = omega_L;
  double send_R = omega_R;

  // ── Watchdog: timeout stop ────────────────────────────────────
  bool cmd_nonzero = (send_L != 0.0 || send_R != 0.0);
  if (cmd_nonzero) {
    last_nonzero_cmd_time_ = time;
    timeout_stop_sent_     = false;
  } else {
    if (last_nonzero_cmd_time_.nanoseconds() > 0 && !timeout_stop_sent_) {
      double elapsed_ms = (time - last_nonzero_cmd_time_).seconds() * 1000.0;
      if (elapsed_ms > cmd_timeout_ms_) {
        std_msgs::msg::Float32MultiArray stop;
        stop.data = {0.0f, 0.0f};
        cmd_pub_->publish(stop);
        last_omega_L_ = 0.0f; last_omega_R_ = 0.0f;
        last_cmd_time_ = time; timeout_stop_sent_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "Watchdog: %.0fms → stop", elapsed_ms);
        return hardware_interface::return_type::OK;
      }
    }
  }

  // ── Snap stop: nonzero → zero ─────────────────────────────────
  if (send_L == 0.0 && send_R == 0.0) {
    bool was_moving = (last_omega_L_ != 0.0f || last_omega_R_ != 0.0f);
    if (was_moving) {
      std_msgs::msg::Float32MultiArray stop;
      stop.data = {0.0f, 0.0f};
      cmd_pub_->publish(stop);
      last_omega_L_ = 0.0f; last_omega_R_ = 0.0f;
      last_cmd_time_ = time;
      RCLCPP_DEBUG(node_->get_logger(),
        "Snap stop fired (threshold=%.3f rad/s)", stop_threshold_);
    }
    return hardware_interface::return_type::OK;
  }

  // ── Rate-limit + change detection ────────────────────────────
  bool time_ok = (last_cmd_time_.nanoseconds() == 0) ||
                 ((time - last_cmd_time_).seconds() >= 1.0 / cmd_publish_rate_);
  bool changed  = std::abs(send_L - static_cast<double>(last_omega_L_)) > 0.005 ||
                  std::abs(send_R - static_cast<double>(last_omega_R_)) > 0.005;

  if (time_ok || changed) {
    std_msgs::msg::Float32MultiArray msg;
    msg.data = {static_cast<float>(send_L), static_cast<float>(send_R)};
    cmd_pub_->publish(msg);
    last_omega_L_  = static_cast<float>(send_L);
    last_omega_R_  = static_cast<float>(send_R);
    last_cmd_time_ = time;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace esp32_hardware

PLUGINLIB_EXPORT_CLASS(
  esp32_hardware::Esp32HardwareInterface,
  hardware_interface::SystemInterface)