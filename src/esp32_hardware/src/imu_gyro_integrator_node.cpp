/*
 * imu_gyro_integrator_node.cpp  —  v3-DriftGuard
 * ─────────────────────────────────────────────────────────────────────────────
 * v3 changes (from v2-ResetService):
 *
 *   [FIX DRIFT]  เพิ่ม yaw wrapping ±π ทุก cycle
 *                เหตุผล: ไม่ wrap → yaw สะสมออกนอก ±π
 *                        EKF ได้ quaternion ที่ orientation ผิดช่วง
 *
 *   [FIX DRIFT]  กรอง gz ด้วย deadzone ที่ node นี้ด้วย (ซ้ำกับ ESP32 ตั้งใจ)
 *                เหตุผล: noise ที่รอดผ่าน ESP32 LPF ยังทำให้ yaw drift
 *                        gz < 0.008 rad/s (≈0.5°/s) ถือว่า noise → ไม่ integrate
 *
 *   [FIX DRIFT]  เพิ่ม gz_bias_estimator แบบง่าย (runniang average ตอน static)
 *                เหตุผล: ESP32 calibrate ครั้งเดียวตอน boot
 *                        temperature drift ทำให้ bias เปลี่ยนระหว่าง run
 *                        เมื่อ |gz| < static_thresh ต่อเนื่aอง 1 วิ → update bias
 *                        ค่า bias ใหม่หักออกจาก gz ก่อน integrate
 *
 *   [FIX JITTER] ใช้ message stamp แทน node time สำหรับ dt
 *                (เหมือน v2 แต่เพิ่ม guard dt > 1/freq_expected)
 *
 *   [KEEP v2]    /imu_integrator/reset service, covariance ทั้งหมด
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * Subscribe: /imu/data_raw  (sensor_msgs/Imu)
 * Publish:   /imu/data      (sensor_msgs/Imu — มี orientation quaternion)
 * Service:   /imu_integrator/reset  (std_srvs/Empty — reset yaw=0 + bias)
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <mutex>

// ── Constants ────────────────────────────────────────────────────────────────
static constexpr double GZ_DEADZONE        = 0.008;   // rad/s — noise floor ที่ node นี้
static constexpr double STATIC_GZ_THRESH   = 0.015;   // rad/s — หุ่นนิ่ง ถ้า gz < นี้
static constexpr double BIAS_UPDATE_ALPHA  = 0.002;   // EMA alpha สำหรับ bias (ช้ามาก)
static constexpr double STATIC_MIN_SECS    = 0.8;     // ต้องนิ่งต่อเนื่องกี่วิถึง update bias
static constexpr double MAX_DT             = 0.3;     // วินาที — dt > นี้ skip (glitch)

class ImuGyroIntegrator : public rclcpp::Node
{
public:
  ImuGyroIntegrator()
  : Node("imu_gyro_integrator"),
    yaw_(0.0),
    gz_bias_(0.0),
    static_accum_(0.0),
    last_stamp_(0, 0, RCL_ROS_TIME),
    initialized_(false)
  {
    auto qos_sub = rclcpp::QoS(rclcpp::KeepLast(50))
                     .best_effort()
                     .durability_volatile();
    auto qos_pub = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();

    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data_raw", qos_sub,
      std::bind(&ImuGyroIntegrator::imuCb, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", qos_pub);

    // Reset service
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/imu_integrator/reset",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) {
        std::lock_guard<std::mutex> lk(yaw_mutex_);
        yaw_         = 0.0;
        gz_bias_     = 0.0;
        static_accum_ = 0.0;
        initialized_ = false;
        RCLCPP_INFO(this->get_logger(),
          "[v3] Yaw + bias reset to 0 — IMU integrator restarted");
      });

    RCLCPP_INFO(this->get_logger(),
      "ImuGyroIntegrator v3-DriftGuard ready");
    RCLCPP_INFO(this->get_logger(),
      "  gz_deadzone=%.4f rad/s  bias_alpha=%.4f  static_thresh=%.4f rad/s",
      GZ_DEADZONE, BIAS_UPDATE_ALPHA, STATIC_GZ_THRESH);
  }

private:
  // ── wrap angle to (-π, π] ──────────────────────────────────────────────────
  static double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time now = msg->header.stamp;
    std::lock_guard<std::mutex> lk(yaw_mutex_);

    if (!initialized_) {
      last_stamp_  = now;
      initialized_ = true;
      return;
    }

    double dt = (now - last_stamp_).seconds();
    last_stamp_ = now;

    // Skip bad dt (clock glitch, resumed from pause, etc.)
    if (dt <= 0.0 || dt > MAX_DT) return;

    // Raw gz หักด้วย bias ที่ประมาณไว้
    double gz_raw = msg->angular_velocity.z - gz_bias_;

    // [FIX DRIFT] deadzone ที่ node นี้ (ซ้ำกับ ESP32 ตั้งใจ — double guard)
    double gz = (std::abs(gz_raw) < GZ_DEADZONE) ? 0.0 : gz_raw;

    // [FIX DRIFT] Bias estimation เมื่อหุ่นนิ่ง
    // ใช้ gz_raw (ก่อน deadzone) เพื่อ estimate bias ที่แท้จริง
    if (std::abs(gz_raw) < STATIC_GZ_THRESH) {
      static_accum_ += dt;
      if (static_accum_ >= STATIC_MIN_SECS) {
        // EMA update bias ด้วย gz_raw ปัจจุบัน
        gz_bias_ = gz_bias_ * (1.0 - BIAS_UPDATE_ALPHA)
                 + gz_raw  * BIAS_UPDATE_ALPHA;
      }
    } else {
      static_accum_ = 0.0;  // reset ทันทีที่หุ่นเริ่มเลี้ยว
    }

    // Integrate yaw
    yaw_ += gz * dt;

    // [FIX DRIFT] Wrap ±π ทุก cycle — ป้องกัน quaternion เพี้ยน
    yaw_ = wrapAngle(yaw_);

    // สร้าง quaternion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);

    sensor_msgs::msg::Imu out = *msg;
    out.header.frame_id = "imu_link";

    out.orientation.x = q.x();
    out.orientation.y = q.y();
    out.orientation.z = q.z();
    out.orientation.w = q.w();

    // Covariance: yaw ใหญ่ขึ้นเล็กน้อยเพื่อให้ EKF ไม่เชื่อมากเกิน
    out.orientation_covariance = {
      1e-3, 0.0,  0.0,
      0.0,  1e-3, 0.0,
      0.0,  0.0,  5e-2   // [v3] เพิ่มจาก 1e-2 → 5e-2 — EKF trust น้อยลง, lidar แก้ได้มากขึ้น
    };

    out.angular_velocity_covariance = {
      0.000025, 0.0,      0.0,
      0.0,      0.000025, 0.0,
      0.0,      0.0,      0.000025
    };

    out.linear_acceleration_covariance = {
      0.0001, 0.0,    0.0,
      0.0,    0.0001, 0.0,
      0.0,    0.0,    0.0001
    };

    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr       reset_srv_;

  std::mutex   yaw_mutex_;
  double       yaw_;
  double       gz_bias_;       // [v3] estimated running bias
  double       static_accum_;  // [v3] seconds of static time accumulated
  rclcpp::Time last_stamp_;
  bool         initialized_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuGyroIntegrator>());
  rclcpp::shutdown();
  return 0;
}