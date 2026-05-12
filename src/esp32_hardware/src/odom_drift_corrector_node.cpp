/*
 * odom_drift_corrector_node.cpp  —  v1-DriftCorrect
 * ─────────────────────────────────────────────────────────────────────────────
 * แก้ปัญหา odom drift สะสมเมื่อวนหลายรอบ → map เพี้ยน
 *
 * วิธีการ:
 *   Cartographer publish tf:  map → odom  (correction transform)
 *   EKF publish tf:           odom → base_footprint
 *   Node นี้ดึง map→odom transform แล้วคำนวณ corrected pose
 *   และ publish /odometry/corrected พร้อม reset service
 *
 * หลักการ:
 *   - ติดตาม drift ระหว่าง odom กับ map frame
 *   - เมื่อ Cartographer loop-close สำเร็จ → map→odom tf เปลี่ยน
 *   - Node นี้ detect การเปลี่ยนแปลงนั้นและ smooth correction ออก
 *   - ป้องกัน pose jump ที่ทำให้ navigation พัง
 *
 * Subscribe:
 *   /tf, /tf_static          — ดึง map→odom (จาก Cartographer)
 *   /odometry/filtered       — EKF odom (odom→base_footprint)
 *
 * Publish:
 *   /odometry/corrected      — odom ที่ correct drift แล้ว (map frame)
 *   /odom_drift/status       — drift magnitude สำหรับ monitoring
 *
 * Service:
 *   /odom_corrector/reset    — reset accumulated drift counter
 *   /odom_corrector/snapshot — บันทึก pose ปัจจุบันเป็น reference
 *
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <mutex>
#include <deque>

// ── Constants ────────────────────────────────────────────────────────────────
// Smoothing: ใช้ EMA กรอง correction เพื่อป้องกัน jump
static constexpr double CORRECTION_ALPHA     = 0.15;   // EMA alpha สำหรับ smooth correction
static constexpr double LOOP_CLOSE_THRESHOLD = 0.08;   // m — ถ้า drift เปลี่ยนเกินนี้ใน 1 cycle → loop-close event
static constexpr double DRIFT_WARN_THRESHOLD = 0.30;   // m — warn เมื่อ accumulated drift เกิน
static constexpr double TF_TIMEOUT_SEC       = 0.5;    // วินาที — timeout ของ tf lookup
static constexpr int    DRIFT_HISTORY_SIZE   = 50;     // จำนวน sample สำหรับ drift trend analysis

class OdomDriftCorrector : public rclcpp::Node
{
public:
  OdomDriftCorrector()
  : Node("odom_drift_corrector"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    correction_x_(0.0),
    correction_y_(0.0),
    correction_yaw_(0.0),
    prev_map_odom_x_(0.0),
    prev_map_odom_y_(0.0),
    prev_map_odom_yaw_(0.0),
    accumulated_drift_(0.0),
    loop_close_count_(0),
    initialized_(false)
  {
    // Parameters
    this->declare_parameter<double>("correction_alpha",     CORRECTION_ALPHA);
    this->declare_parameter<double>("loop_close_threshold", LOOP_CLOSE_THRESHOLD);
    this->declare_parameter<double>("drift_warn_threshold", DRIFT_WARN_THRESHOLD);
    this->declare_parameter<std::string>("map_frame",       "map");
    this->declare_parameter<std::string>("odom_frame",      "odom");
    this->declare_parameter<std::string>("base_frame",      "base_footprint");

    alpha_           = this->get_parameter("correction_alpha").as_double();
    lc_threshold_    = this->get_parameter("loop_close_threshold").as_double();
    warn_threshold_  = this->get_parameter("drift_warn_threshold").as_double();
    map_frame_       = this->get_parameter("map_frame").as_string();
    odom_frame_      = this->get_parameter("odom_frame").as_string();
    base_frame_      = this->get_parameter("base_frame").as_string();

    // QoS
    auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto qos_best     = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // Subscribe filtered odom จาก EKF
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/filtered", qos_best,
      std::bind(&OdomDriftCorrector::odomCb, this, std::placeholders::_1));

    // Publish
    corrected_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odometry/corrected", qos_reliable);
    drift_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      "/odom_drift/status", qos_reliable);

    // Services
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/odom_corrector/reset",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) {
        std::lock_guard<std::mutex> lk(mutex_);
        correction_x_      = 0.0;
        correction_y_      = 0.0;
        correction_yaw_    = 0.0;
        accumulated_drift_ = 0.0;
        loop_close_count_  = 0;
        initialized_       = false;
        drift_history_.clear();
        RCLCPP_INFO(this->get_logger(),
          "[OdomDriftCorrector] Reset: correction zeroed, drift history cleared");
      });

    snapshot_srv_ = this->create_service<std_srvs::srv::Empty>(
      "/odom_corrector/snapshot",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) {
        std::lock_guard<std::mutex> lk(mutex_);
        RCLCPP_INFO(this->get_logger(),
          "[OdomDriftCorrector] Snapshot: drift=%.4fm, loop_close_count=%d, "
          "correction=(%.4f, %.4f, %.4f°)",
          accumulated_drift_, loop_close_count_,
          correction_x_, correction_y_, correction_yaw_ * 180.0 / M_PI);
      });

    // Timer สำหรับ monitor drift trend (1 Hz)
    monitor_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&OdomDriftCorrector::monitorDrift, this));

    RCLCPP_INFO(this->get_logger(),
      "OdomDriftCorrector v1-DriftCorrect ready");
    RCLCPP_INFO(this->get_logger(),
      "  frames: map=%s odom=%s base=%s",
      map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(),
      "  alpha=%.3f  lc_threshold=%.3fm  warn_threshold=%.3fm",
      alpha_, lc_threshold_, warn_threshold_);
  }

private:
  // ── Odom callback ──────────────────────────────────────────────────────────
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mutex_);

    // ดึง map→odom transform จาก Cartographer
    geometry_msgs::msg::TransformStamped tf_map_odom;
    try {
      tf_map_odom = tf_buffer_.lookupTransform(
        map_frame_, odom_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(TF_TIMEOUT_SEC));
    } catch (const tf2::TransformException & e) {
      // Cartographer ยังไม่ได้ publish tf — ใช้ odom เดิมไปก่อน
      corrected_pub_->publish(*msg);
      return;
    }

    double map_odom_x   = tf_map_odom.transform.translation.x;
    double map_odom_y   = tf_map_odom.transform.translation.y;
    double map_odom_yaw = getYaw(tf_map_odom.transform.rotation);

    if (!initialized_) {
      prev_map_odom_x_   = map_odom_x;
      prev_map_odom_y_   = map_odom_y;
      prev_map_odom_yaw_ = map_odom_yaw;
      initialized_ = true;
      corrected_pub_->publish(*msg);
      return;
    }

    // ตรวจจับ loop-close event: map→odom เปลี่ยนกระทันหัน
    double dx      = map_odom_x   - prev_map_odom_x_;
    double dy      = map_odom_y   - prev_map_odom_y_;
    double d_drift = std::sqrt(dx*dx + dy*dy);

    if (d_drift > lc_threshold_) {
      loop_close_count_++;
      accumulated_drift_ += d_drift;

      RCLCPP_INFO(this->get_logger(),
        "[LOOP-CLOSE #%d] map→odom shifted %.4fm (total drift=%.4fm)",
        loop_close_count_, d_drift, accumulated_drift_);

      // เก็บ history
      drift_history_.push_back(d_drift);
      if ((int)drift_history_.size() > DRIFT_HISTORY_SIZE)
        drift_history_.pop_front();
    }

    // Smooth correction โดยใช้ EMA
    // correction = ส่วนต่างระหว่าง map→odom ปัจจุบัน กับที่ smooth แล้ว
    correction_x_   = alpha_ * map_odom_x   + (1.0 - alpha_) * correction_x_;
    correction_y_   = alpha_ * map_odom_y   + (1.0 - alpha_) * correction_y_;
    correction_yaw_ = alpha_ * map_odom_yaw + (1.0 - alpha_) * correction_yaw_;

    prev_map_odom_x_   = map_odom_x;
    prev_map_odom_y_   = map_odom_y;
    prev_map_odom_yaw_ = map_odom_yaw;

    // Apply correction: แปลง odom pose เข้า map frame
    nav_msgs::msg::Odometry corrected = *msg;
    corrected.header.frame_id = map_frame_;

    // Transform pose จาก odom frame → map frame
    // P_map = T(map→odom) * P_odom
    double ox = msg->pose.pose.position.x;
    double oy = msg->pose.pose.position.y;
    double cos_c = std::cos(correction_yaw_);
    double sin_c = std::sin(correction_yaw_);

    corrected.pose.pose.position.x = correction_x_ + cos_c * ox - sin_c * oy;
    corrected.pose.pose.position.y = correction_y_ + sin_c * ox + cos_c * oy;
    corrected.pose.pose.position.z = msg->pose.pose.position.z;

    // แก้ yaw
    double orig_yaw = getYaw(msg->pose.pose.orientation);
    double corr_yaw = wrapAngle(orig_yaw + correction_yaw_);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, corr_yaw);
    corrected.pose.pose.orientation.x = q.x();
    corrected.pose.pose.orientation.y = q.y();
    corrected.pose.pose.orientation.z = q.z();
    corrected.pose.pose.orientation.w = q.w();

    corrected_pub_->publish(corrected);

    // Publish drift magnitude
    std_msgs::msg::Float64 drift_msg;
    drift_msg.data = accumulated_drift_;
    drift_pub_->publish(drift_msg);

    // Warning ถ้า drift สูง
    if (accumulated_drift_ > warn_threshold_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "[OdomDriftCorrector] Accumulated drift=%.4fm > threshold=%.4fm "
        "(loop_close_count=%d)",
        accumulated_drift_, warn_threshold_, loop_close_count_);
    }
  }

  // ── Monitor drift trend ────────────────────────────────────────────────────
  void monitorDrift()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (drift_history_.empty()) return;

    // คำนวณ average drift per loop-close
    double sum = 0.0;
    for (auto v : drift_history_) sum += v;
    double avg = sum / drift_history_.size();

    if (avg > 0.05) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
        "[DriftMonitor] avg_drift_per_lc=%.4fm  total=%.4fm  lc_count=%d",
        avg, accumulated_drift_, loop_close_count_);
    }
  }

  // ── Helpers ────────────────────────────────────────────────────────────────
  static double getYaw(const geometry_msgs::msg::Quaternion & q) {
    tf2::Quaternion tq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
    return yaw;
  }

  static double wrapAngle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // ── Members ───────────────────────────────────────────────────────────────
  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex                 mutex_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    corrected_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr     drift_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr         reset_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr         snapshot_srv_;
  rclcpp::TimerBase::SharedPtr                             monitor_timer_;

  // Correction state (EMA-smoothed map→odom transform)
  double correction_x_;
  double correction_y_;
  double correction_yaw_;

  // Previous map→odom (สำหรับ detect loop-close)
  double prev_map_odom_x_;
  double prev_map_odom_y_;
  double prev_map_odom_yaw_;

  // Drift tracking
  double accumulated_drift_;
  int    loop_close_count_;
  bool   initialized_;
  std::deque<double> drift_history_;

  // Params
  double      alpha_;
  double      lc_threshold_;
  double      warn_threshold_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomDriftCorrector>());
  rclcpp::shutdown();
  return 0;
}
