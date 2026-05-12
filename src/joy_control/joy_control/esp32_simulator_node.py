#!/usr/bin/env python3
"""
esp32_simulator_node.py  —  v7
─────────────────────────────────────────────────────────────────────────────
แก้ไขจาก v6:

[ROOT CAUSE FIX 1] ใน GazeboSystem mode ห้าม publish /topic_based_joint_commands
  v6 BUG: esp32_simulator publish /topic_based_joint_commands → GazeboSystem ไม่รับ
          เพราะ URDF ใช้ <plugin>gazebo_ros2_control/GazeboSystem</plugin>
          ไม่ใช่ TopicBasedSystem → ล้อไม่หมุน TF ล้อไม่ขึ้น

  v7 FIX: เพิ่ม param sim_gazebo_system=True (default)
          ถ้า True → ไม่ publish /topic_based_joint_commands
          diff_drive_controller ควบคุมล้อผ่าน GazeboSystem โดยตรงอยู่แล้ว
          esp32_simulator ทำหน้าที่แค่: subscribe cmd_vel + relay IMU เท่านั้น

[ROOT CAUSE FIX 2] RuntimeError: Unable to convert call argument
  v6 BUG: joint_state_cb รับ JointState ที่มี velocity=[] (empty list)
          → msg.velocity[i] index out of range → RuntimeError
  v7 FIX: guard len(msg.velocity) > 0 และ check i < len(msg.velocity)
          (v6 มีแล้วแต่ยังพลาดกรณี velocity list สั้นกว่า name list)

[คง]   PID simulation ยังทำงานปกติ (จำลอง firmware behavior)
       IMU fallback zeros ถ้า Gazebo IMU ไม่ส่ง
       Watchdog timeout 500ms
─────────────────────────────────────────────────────────────────────────────

Data flow (v7 GazeboSystem mode):
  Joy → twist_mux → /diff_drive_controller/cmd_vel_unstamped
    → diff_drive_controller (GazeboSystem)
      → Gazebo physics applies wheel velocity DIRECTLY
        → /gazebo/joint_states → joint_state_broadcaster → /joint_states
          → robot_state_publisher → TF ล้อขึ้น ✓
          → diff_drive_controller → /diff_drive_controller/odom

  esp32_simulator (v7 sim mode):
    - subscribe cmd_vel → log + simulate PID (firmware behavior only)
    - subscribe /imu/data_raw → relay to /imu_raw
    - publish IMU fallback zeros ถ้า Gazebo IMU ไม่ส่ง
    - ไม่ publish /topic_based_joint_commands (GazeboSystem ไม่รับ)
─────────────────────────────────────────────────────────────────────────────
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist


class Esp32SimulatorNode(Node):
    def __init__(self):
        super().__init__('esp32_simulator')

        # ── Parameters (match firmware v33r) ──────────────────────────────
        self.declare_parameter('wheel_radius',       0.0826)
        self.declare_parameter('wheel_separation',   0.624)
        self.declare_parameter('max_rpm',            60.0)
        self.declare_parameter('ff_pwm',             24.0)
        self.declare_parameter('kp',                 0.50)
        self.declare_parameter('ki',                 0.120)
        self.declare_parameter('kd',                 0.00)
        self.declare_parameter('sum_clamp',          20.0)
        self.declare_parameter('rpm_filt_alpha',     0.50)
        self.declare_parameter('preload_min',        10.0)
        self.declare_parameter('preload_max',        25.0)
        self.declare_parameter('startup_ramp_ticks', 2)
        self.declare_parameter('pwm_deadband',       5.0)
        self.declare_parameter('pwm_max',            200.0)
        self.declare_parameter('cmd_timeout_ms',     500.0)
        self.declare_parameter('state_publish_rate', 20.0)
        self.declare_parameter('imu_publish_rate',   50.0)
        self.declare_parameter('joint_state_topic',  '/joint_states')
        self.declare_parameter('imu_timeout_s',      5.0)
        # [FIX 1] ใน GazeboSystem mode ไม่ publish joint commands
        # set True เมื่อ URDF ใช้ gazebo_ros2_control/GazeboSystem
        # set False เมื่อ URDF ใช้ TopicBasedSystem
        self.declare_parameter('sim_gazebo_system',  True)

        r = self.get_parameter
        self.wheel_radius       = r('wheel_radius').value
        self.wheel_sep          = r('wheel_separation').value
        self.max_rpm            = r('max_rpm').value
        self.ff_pwm             = r('ff_pwm').value
        self.kp                 = r('kp').value
        self.ki                 = r('ki').value
        self.kd                 = r('kd').value
        self.sum_clamp          = r('sum_clamp').value
        self.filt_alpha         = r('rpm_filt_alpha').value
        self.preload_min        = r('preload_min').value
        self.preload_max        = r('preload_max').value
        self.startup_ticks      = r('startup_ramp_ticks').value
        self.pwm_deadband       = r('pwm_deadband').value
        self.pwm_max            = r('pwm_max').value
        self.cmd_timeout_ms     = r('cmd_timeout_ms').value
        state_rate              = r('state_publish_rate').value
        imu_rate                = r('imu_publish_rate').value
        js_topic                = r('joint_state_topic').value
        self.imu_timeout_s      = r('imu_timeout_s').value
        self.sim_gazebo_system  = r('sim_gazebo_system').value

        # max_omega จาก max_rpm (6.28 rad/s ที่ 60 rpm)
        self.max_omega = self.max_rpm * 2.0 * math.pi / 60.0

        # ── Motor state ────────────────────────────────────────────────────
        self.cmd_omega_L       = 0.0
        self.cmd_omega_R       = 0.0
        self.cmd_rpm_L         = 0.0
        self.cmd_rpm_R         = 0.0
        self.side_L_active     = False
        self.side_R_active     = False
        self.last_cmd_time     = self.get_clock().now()

        self.true_rpm_L = 0.0
        self.true_rpm_R = 0.0
        self.joint_states_received = False

        self.rpm_L_filt = 0.0
        self.rpm_R_filt = 0.0

        self.err_L = 0.0;  self.prev_err_L = 0.0;  self.sum_err_L = 0.0
        self.err_R = 0.0;  self.prev_err_R = 0.0;  self.sum_err_R = 0.0
        self.pwm_L = 0.0
        self.pwm_R = 0.0

        self.startup_skip_count = 0

        self.left_joint  = 'LFHover_Wheel_joint'
        self.right_joint = 'RTHover_Wheel_joint'

        self._cmd_recv_count = 0

        self.last_imu_time   = self.get_clock().now()
        self.imu_fallback_on = False

        # ── Publishers ─────────────────────────────────────────────────────
        self.motor_pub   = self.create_publisher(Float32MultiArray, '/motor_state', 10)
        self.imu_raw_pub = self.create_publisher(Float32MultiArray, '/imu_raw', 50)
        self.imu_pub     = self.create_publisher(Imu, '/imu/data_raw', 50)

        # [FIX 1] publish joint_cmd เฉพาะ TopicBasedSystem เท่านั้น
        if not self.sim_gazebo_system:
            self.joint_cmd_pub = self.create_publisher(
                JointState, '/topic_based_joint_commands', 10)
            self.get_logger().info('Mode: TopicBasedSystem → will publish /topic_based_joint_commands')
        else:
            self.joint_cmd_pub = None
            self.get_logger().info(
                'Mode: GazeboSystem → diff_drive_controller controls wheels directly\n'
                '  esp32_simulator role: cmd_vel monitor + IMU relay only')

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            Twist, '/diff_drive_controller/cmd_vel_unstamped',
            self.cmd_vel_cb, 10)
        self.create_subscription(
            Float32MultiArray, '/wheel_cmd', self.wheel_cmd_cb, 10)
        self.create_subscription(
            JointState, js_topic, self.joint_state_cb, 10)
        self.create_subscription(
            Imu, '/imu/data_raw', self.imu_cb, 50)

        # ── Timers ─────────────────────────────────────────────────────────
        self.create_timer(1.0 / state_rate, self.motor_pid_task)
        self.create_timer(1.0 / imu_rate,   self.imu_task)
        self.create_timer(1.0,              self.check_imu_timeout)

        self.get_logger().info(
            f'ESP32 Simulator v7 | firmware v33r sync | GazeboSystem={self.sim_gazebo_system}\n'
            f'  FF={self.ff_pwm} KP={self.kp} KI={self.ki} KD={self.kd}\n'
            f'  SUM_CLAMP={self.sum_clamp} ALPHA={self.filt_alpha}\n'
            f'  PRELOAD=[{self.preload_min},{self.preload_max}] '
            f'STARTUP_TICKS={self.startup_ticks} '
            f'PWM_DEADBAND={self.pwm_deadband}\n'
            f'  max_omega={self.max_omega:.3f} rad/s')

    # ══════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════

    def cmd_vel_cb(self, msg: Twist):
        v  = msg.linear.x
        w  = msg.angular.z
        oL = (v - w * self.wheel_sep / 2.0) / self.wheel_radius
        oR = (v + w * self.wheel_sep / 2.0) / self.wheel_radius
        self._set_cmd(oL, oR)

        self._cmd_recv_count += 1
        if self._cmd_recv_count % 20 == 1:
            self.get_logger().info(
                f'cmd_vel: v={v:.3f} w={w:.3f} → oL={oL:.3f} oR={oR:.3f} rad/s',
                throttle_duration_sec=1.0)

    def wheel_cmd_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        self._set_cmd(float(msg.data[0]), float(msg.data[1]))

    def _set_cmd(self, oL: float, oR: float):
        self.cmd_omega_L   = oL
        self.cmd_omega_R   = oR
        self.last_cmd_time = self.get_clock().now()

    def joint_state_cb(self, msg: JointState):
        """
        [FIX 2] guard ป้องกัน RuntimeError: Unable to convert call argument
        กรณี velocity list ว่าง หรือสั้นกว่า name list
        """
        if not msg.velocity or len(msg.velocity) == 0:
            return
        self.joint_states_received = True
        for i, name in enumerate(msg.name):
            if i >= len(msg.velocity):
                break
            try:
                rpm = float(msg.velocity[i]) * 60.0 / (2.0 * math.pi)
            except (TypeError, ValueError):
                continue
            if name == self.left_joint:
                self.true_rpm_L = rpm
            elif name == self.right_joint:
                self.true_rpm_R = rpm

    def imu_cb(self, msg: Imu):
        self.last_imu_time = self.get_clock().now()
        if self.imu_fallback_on:
            self.imu_fallback_on = False
            self.get_logger().info('Real IMU received — fallback disabled')
        raw = Float32MultiArray()
        raw.data = [
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z),
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        ]
        self.imu_raw_pub.publish(raw)

    # ══════════════════════════════════════════════════════════════════
    # IMU Fallback — 50 Hz
    # ══════════════════════════════════════════════════════════════════

    def check_imu_timeout(self):
        elapsed = (self.get_clock().now() - self.last_imu_time).nanoseconds / 1e9
        if elapsed > self.imu_timeout_s and not self.imu_fallback_on:
            self.imu_fallback_on = True
            self.get_logger().warn(
                f'No IMU for {elapsed:.1f}s → publishing zeros to /imu/data_raw')

    def imu_task(self):
        if not self.imu_fallback_on:
            return
        now = self.get_clock().now().to_msg()
        imu = Imu()
        imu.header.stamp             = now
        imu.header.frame_id          = 'imu_link'
        imu.linear_acceleration.z    = 9.81
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity_covariance = [
            0.000025, 0, 0, 0, 0.000025, 0, 0, 0, 0.000025]
        imu.linear_acceleration_covariance = [
            0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001]
        self.imu_pub.publish(imu)
        raw = Float32MultiArray()
        raw.data = [0.0, 0.0, 9.81, 0.0, 0.0, 0.0]
        self.imu_raw_pub.publish(raw)

    # ══════════════════════════════════════════════════════════════════
    # applyStop
    # ══════════════════════════════════════════════════════════════════

    def _apply_stop_L(self):
        self.rpm_L_filt    = 0.0; self.pwm_L       = 0.0
        self.sum_err_L     = 0.0; self.err_L        = 0.0
        self.prev_err_L    = 0.0; self.side_L_active = False
        self.cmd_rpm_L     = 0.0

    def _apply_stop_R(self):
        self.rpm_R_filt    = 0.0; self.pwm_R       = 0.0
        self.sum_err_R     = 0.0; self.err_R        = 0.0
        self.prev_err_R    = 0.0; self.side_R_active = False
        self.cmd_rpm_R     = 0.0

    # ══════════════════════════════════════════════════════════════════
    # processCmd
    # ══════════════════════════════════════════════════════════════════

    def _clamp_rpm(self, omega: float) -> float:
        rpm = omega * 60.0 / (2.0 * math.pi)
        return max(-self.max_rpm, min(self.max_rpm, rpm))

    def _process_cmd(self, oL: float, oR: float):
        new_L = abs(oL) > 0.01
        new_R = abs(oR) > 0.01

        if not new_L:
            self._apply_stop_L()
        if not new_R:
            self._apply_stop_R()

        was_stop_L = not self.side_L_active
        was_stop_R = not self.side_R_active

        if new_L:
            new_rpm_L = self._clamp_rpm(oL)
            if self.cmd_rpm_L != 0.0 and (new_rpm_L > 0) != (self.cmd_rpm_L > 0):
                self.sum_err_L = 0.0; self.prev_err_L = 0.0
                self.err_L     = 0.0; self.rpm_L_filt = 0.0
            self.side_L_active = True
            self.cmd_rpm_L     = new_rpm_L

        if new_R:
            new_rpm_R = self._clamp_rpm(oR)
            if self.cmd_rpm_R != 0.0 and (new_rpm_R > 0) != (self.cmd_rpm_R > 0):
                self.sum_err_R = 0.0; self.prev_err_R = 0.0
                self.err_R     = 0.0; self.rpm_R_filt = 0.0
            self.side_R_active = True
            self.cmd_rpm_R     = new_rpm_R

        fresh_start = (new_L and was_stop_L) or (new_R and was_stop_R)
        if fresh_start:
            self.startup_skip_count = self.startup_ticks
            self.sum_err_L = 0.0; self.prev_err_L = 0.0
            self.err_L     = 0.0; self.rpm_L_filt = 0.0
            self.sum_err_R = 0.0; self.prev_err_R = 0.0
            self.err_R     = 0.0; self.rpm_R_filt = 0.0

            if self.side_L_active:
                pre_L = self.ff_pwm * (abs(self.cmd_rpm_L) / self.max_rpm)
                self.pwm_L = max(self.preload_min, min(self.preload_max, pre_L))
            if self.side_R_active:
                pre_R = self.ff_pwm * (abs(self.cmd_rpm_R) / self.max_rpm)
                self.pwm_R = max(self.preload_min, min(self.preload_max, pre_R))

    # ══════════════════════════════════════════════════════════════════
    # [FIX 1] _publish_joint_cmd — เฉพาะ TopicBasedSystem เท่านั้น
    # ══════════════════════════════════════════════════════════════════

    def _publish_joint_cmd(self):
        """
        [FIX 1] publish เฉพาะเมื่อ sim_gazebo_system=False (TopicBasedSystem)
        GazeboSystem: diff_drive_controller ส่ง velocity command ไป hardware interface โดยตรง
                      ผ่าน gazebo_ros2_control plugin → ไม่ต้องการ /topic_based_joint_commands
        """
        if self.joint_cmd_pub is None:
            return  # GazeboSystem mode: ไม่ publish

        now = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = now
        msg.name = [self.left_joint, self.right_joint]

        omega_L = 0.0
        if self.side_L_active and abs(self.cmd_omega_L) > 0.01:
            omega_L = max(-self.max_omega, min(self.max_omega, self.cmd_omega_L))

        omega_R = 0.0
        if self.side_R_active and abs(self.cmd_omega_R) > 0.01:
            omega_R = max(-self.max_omega, min(self.max_omega, self.cmd_omega_R))

        msg.velocity = [omega_L, omega_R]
        msg.position = [0.0, 0.0]
        self.joint_cmd_pub.publish(msg)

    # ══════════════════════════════════════════════════════════════════
    # motorPidTask — จำลอง firmware behavior + publish state
    # ══════════════════════════════════════════════════════════════════

    def motor_pid_task(self):
        now = self.get_clock().now()

        if ((self.side_L_active or self.side_R_active) and
                (now - self.last_cmd_time).nanoseconds / 1e6 > self.cmd_timeout_ms):
            self.cmd_omega_L = 0.0
            self.cmd_omega_R = 0.0
            self.get_logger().warn('WARN:TIMEOUT', throttle_duration_sec=5.0)

        self._process_cmd(self.cmd_omega_L, self.cmd_omega_R)

        if self.joint_states_received:
            mL = abs(self.true_rpm_L) if self.side_L_active else 0.0
            mR = abs(self.true_rpm_R) if self.side_R_active else 0.0
        else:
            mL = abs(self.cmd_rpm_L) if self.side_L_active else 0.0
            mR = abs(self.cmd_rpm_R) if self.side_R_active else 0.0

        a = self.filt_alpha
        self.rpm_L_filt = a * mL + (1.0 - a) * self.rpm_L_filt
        self.rpm_R_filt = a * mR + (1.0 - a) * self.rpm_R_filt

        sL = 1.0 if self.cmd_rpm_L >= 0 else -1.0
        sR = 1.0 if self.cmd_rpm_R >= 0 else -1.0
        rL = sL * mL
        rR = sR * mR

        if self.startup_skip_count > 0:
            self.startup_skip_count -= 1
            self._publish_joint_cmd()
            self._publish_state(rL, rR)
            return

        # ── PID L ──────────────────────────────────────────────────────────
        if not self.side_L_active:
            self.pwm_L = 0.0
        else:
            tgt_L        = abs(self.cmd_rpm_L)
            self.err_L   = tgt_L - self.rpm_L_filt
            dL           = self.err_L - self.prev_err_L
            ns_L         = max(-self.sum_clamp,
                               min(self.sum_clamp, self.sum_err_L + self.err_L))
            ff_L         = self.ff_pwm * (tgt_L / self.max_rpm)
            out_L        = ff_L + self.kp * self.err_L + self.ki * ns_L + self.kd * dL
            if 0.0 < out_L < self.pwm_max:
                self.sum_err_L = ns_L
            self.pwm_L      = max(0.0, min(self.pwm_max, out_L))
            self.prev_err_L = self.err_L
            if self.pwm_L < self.pwm_deadband:
                self.pwm_L = 0.0

        # ── PID R ──────────────────────────────────────────────────────────
        if not self.side_R_active:
            self.pwm_R = 0.0
        else:
            tgt_R        = abs(self.cmd_rpm_R)
            self.err_R   = tgt_R - self.rpm_R_filt
            dR           = self.err_R - self.prev_err_R
            ns_R         = max(-self.sum_clamp,
                               min(self.sum_clamp, self.sum_err_R + self.err_R))
            ff_R         = self.ff_pwm * (tgt_R / self.max_rpm)
            out_R        = ff_R + self.kp * self.err_R + self.ki * ns_R + self.kd * dR
            if 0.0 < out_R < self.pwm_max:
                self.sum_err_R = ns_R
            self.pwm_R      = max(0.0, min(self.pwm_max, out_R))
            self.prev_err_R = self.err_R
            if self.pwm_R < self.pwm_deadband:
                self.pwm_R = 0.0

        # [FIX 1] ใน GazeboSystem mode: ไม่ publish (no-op)
        self._publish_joint_cmd()

        self._publish_state(rL, rR)

    def _publish_state(self, rL: float, rR: float):
        msg = Float32MultiArray()
        msg.data = [float(rL), float(rR), float(self.pwm_L), float(self.pwm_R)]
        self.motor_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Esp32SimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()