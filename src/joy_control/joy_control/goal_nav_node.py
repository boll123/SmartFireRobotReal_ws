#!/usr/bin/env python3
# goal_nav_node.py  —  v13-LongRange
# สิ่งที่แก้ไขจาก v12:
#   - XY_CLOSE_THRESH: 0.5 → 30.0  (รองรับระยะสูงสุด 30m)
#   - เพิ่ม PLAN_RETRY: ถ้า ABORTED ไกล → retry อัตโนมัติ MAX_PLAN_RETRY ครั้ง
#   - retry delay 2.0s (รอ costmap อัปเดต)

import math
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
)
from nav2_msgs.action import NavigateToPose

YAW_THRESH       = 0.06
XY_CLOSE_THRESH  = 30.0   # [FIX] รองรับระยะสูงสุด 30m
MAX_PLAN_RETRY   = 3       # retry ถ้า planner fail
PLAN_RETRY_DELAY = 2.0     # วินาที รอก่อน retry
SPIN_ANG_VEL     = 0.3
MAX_CORRECT_ITER = 8
SETTLE_WAIT_SEC  = 1.2
SETTLE_STABLE    = 4
SETTLE_TIMEOUT   = 3.0
AMCL_STABLE_DEG  = 1.5


def yaw_to_quaternion(yaw_rad):
    q = Quaternion()
    q.x = 0.0; q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


def quaternion_to_yaw(q):
    return 2.0 * math.atan2(q.z, q.w)


def normalize_angle(a):
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


class GoalNavNode(Node):
    def __init__(self):
        super().__init__('goal_nav_node')
        self._cb_group = ReentrantCallbackGroup()

        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self._sub_goal = self.create_subscription(
            Quaternion, '/goal_xy', self._goal_callback, 10,
            callback_group=self._cb_group,
        )
        self._amcl_x   = 0.0
        self._amcl_y   = 0.0
        self._amcl_yaw = 0.0
        self._sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_callback, 10,
            callback_group=self._cb_group,
        )

        self._is_navigating       = False
        self._current_goal_handle = None
        self._target_x            = 0.0
        self._target_y            = 0.0
        self._target_yaw_rad      = 0.0
        self._plan_retry_count    = 0

        # SMC state
        self._smc_timer       = None
        self._smc_phase       = 'idle'
        self._smc_elapsed     = 0.0
        self._smc_spin_dur    = 0.0
        self._smc_spin_dir    = 1.0
        self._smc_iter        = 0
        self._smc_settle_buf  = []

        self.get_logger().info(
            '[GoalNavNode v13-LongRange] พร้อมรับ /goal_xy\n'
            f'  XY_CLOSE_THRESH={XY_CLOSE_THRESH}m  '
            f'MAX_PLAN_RETRY={MAX_PLAN_RETRY}  '
            f'YAW_THRESH={math.degrees(YAW_THRESH):.1f}°'
        )

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        self._amcl_x   = msg.pose.pose.position.x
        self._amcl_y   = msg.pose.pose.position.y
        self._amcl_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self._smc_phase == 'settling':
            self._smc_settle_buf.append(self._amcl_yaw)

    def _goal_callback(self, msg: Quaternion):
        self._cancel_smc()
        if self._is_navigating and self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._is_navigating = False

        self._target_x          = msg.x
        self._target_y          = msg.y
        self._target_yaw_rad    = math.radians(msg.w)
        self._plan_retry_count  = 0

        self.get_logger().info(
            f'[GoalNavNode] รับ goal: x={msg.x:.3f}  y={msg.y:.3f}  yaw={msg.w:.1f}°'
        )
        self._send_goal(msg.x, msg.y, self._target_yaw_rad)

    def _send_goal(self, x, y, yaw_rad):
        while not self._action_client.wait_for_server(timeout_sec=2.0):
            if not rclpy.ok():
                return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose                  = PoseStamped()
        goal_msg.pose.header.frame_id  = 'map'
        goal_msg.pose.header.stamp     = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x  = x
        goal_msg.pose.pose.position.y  = y
        goal_msg.pose.pose.position.z  = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw_rad)

        future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)
        self._is_navigating = True

    def _goal_response_callback(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('[GoalNavNode] Goal ถูกปฏิเสธ!')
            self._is_navigating = False
            return
        self.get_logger().info('[GoalNavNode] Goal ถูกตอบรับ → เดินทาง...')
        self._current_goal_handle = gh
        gh.get_result_async().add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        if hasattr(feedback_msg.feedback, 'distance_remaining'):
            self.get_logger().info(
                f'[GoalNavNode] ระยะที่เหลือ: '
                f'{feedback_msg.feedback.distance_remaining:.2f} m',
                throttle_duration_sec=2.0,
            )

    def _result_callback(self, future):
        status = future.result().status
        STATUS_NAMES = {4: 'SUCCEEDED ✓', 5: 'CANCELED', 6: 'ABORTED ✗'}

        self._is_navigating       = False
        self._current_goal_handle = None

        dx      = self._target_x - self._amcl_x
        dy      = self._target_y - self._amcl_y
        xy_dist = math.sqrt(dx * dx + dy * dy)

        self.get_logger().info(
            f'[GoalNavNode] Nav: {STATUS_NAMES.get(status, str(status))}  '
            f'dist={xy_dist:.2f}m'
        )

        if status == 4:
            # ถึงแล้ว → แก้ yaw
            self._smc_iter = 0
            self._smc_start_settle()

        elif status == 6:
            if xy_dist <= XY_CLOSE_THRESH:
                # ใกล้พอ → แก้ yaw (รวมถึงกรณีถึงแต่ ABORTED)
                self.get_logger().warn(
                    f'[GoalNavNode] ABORTED dist={xy_dist:.2f}m ≤ {XY_CLOSE_THRESH}m '
                    f'→ แก้ yaw'
                )
                self._smc_iter = 0
                self._smc_start_settle()
            else:
                # ไกล → ตรวจว่าเป็น planner fail หรือเปล่า → retry
                if self._plan_retry_count < MAX_PLAN_RETRY:
                    self._plan_retry_count += 1
                    self.get_logger().warn(
                        f'[GoalNavNode] planner fail → retry '
                        f'#{self._plan_retry_count}/{MAX_PLAN_RETRY} '
                        f'(รอ {PLAN_RETRY_DELAY}s)'
                    )
                    # รอแล้ว retry
                    retry_timer = self.create_timer(
                        PLAN_RETRY_DELAY,
                        lambda: self._do_plan_retry(retry_timer),
                        callback_group=self._cb_group,
                    )
                else:
                    self.get_logger().error(
                        f'[GoalNavNode] planner fail หมด retry  '
                        f'dist={xy_dist:.2f}m'
                    )

    def _do_plan_retry(self, timer):
        timer.cancel()
        self.get_logger().info(
            f'[GoalNavNode] retry goal: '
            f'x={self._target_x:.2f}  y={self._target_y:.2f}  '
            f'yaw={math.degrees(self._target_yaw_rad):.1f}°'
        )
        self._send_goal(self._target_x, self._target_y, self._target_yaw_rad)

    # ══════════════════════════════════════════════════════════════
    # Stop-Measure-Correct (จาก v12 ไม่เปลี่ยน)
    # ══════════════════════════════════════════════════════════════

    def _smc_start_settle(self):
        self._cmd_vel_pub.publish(Twist())
        self._smc_phase      = 'settling'
        self._smc_elapsed    = 0.0
        self._smc_settle_buf = []
        self.get_logger().info(
            f'[SMC] settle...  amcl={math.degrees(self._amcl_yaw):.1f}°'
        )
        self._smc_timer = self.create_timer(
            1.0 / 20.0, self._smc_settle_tick,
            callback_group=self._cb_group,
        )

    def _smc_settle_tick(self):
        self._smc_elapsed += 1.0 / 20.0

        amcl_stable = False
        if len(self._smc_settle_buf) >= SETTLE_STABLE:
            last_n = self._smc_settle_buf[-SETTLE_STABLE:]
            spread = max(last_n) - min(last_n)
            amcl_stable = spread < math.radians(AMCL_STABLE_DEG)

        settled = (
            self._smc_elapsed >= SETTLE_WAIT_SEC and amcl_stable
        ) or self._smc_elapsed >= SETTLE_TIMEOUT

        if not settled:
            return

        self._smc_timer.cancel()
        self._smc_timer = None
        self._smc_phase = 'idle'

        measured_yaw = self._amcl_yaw
        error = normalize_angle(self._target_yaw_rad - measured_yaw)

        self.get_logger().info(
            f'[SMC] วัด: target={math.degrees(self._target_yaw_rad):.1f}°  '
            f'amcl={math.degrees(measured_yaw):.1f}°  '
            f'err={math.degrees(error):.1f}°  '
            f'iter={self._smc_iter}/{MAX_CORRECT_ITER}'
        )

        if abs(error) <= YAW_THRESH:
            self.get_logger().info(
                f'[GoalNavNode] ✓ yaw OK!  '
                f'amcl={math.degrees(measured_yaw):.1f}°  '
                f'err={math.degrees(error):.1f}°'
            )
            return

        if self._smc_iter >= MAX_CORRECT_ITER:
            self.get_logger().warn(
                f'[GoalNavNode] หมด iter  '
                f'final err={math.degrees(error):.1f}°'
            )
            return

        spin_dur = abs(error) / SPIN_ANG_VEL
        spin_dir = 1.0 if error > 0 else -1.0

        self.get_logger().info(
            f'[SMC] หมุน {math.degrees(error):.1f}°  '
            f'dur={spin_dur:.2f}s  '
            f'dir={"CCW" if spin_dir > 0 else "CW"}'
        )

        self._smc_iter    += 1
        self._smc_phase    = 'spinning'
        self._smc_elapsed  = 0.0
        self._smc_spin_dur = spin_dur
        self._smc_spin_dir = spin_dir

        self._smc_timer = self.create_timer(
            1.0 / 20.0, self._smc_spin_tick,
            callback_group=self._cb_group,
        )

    def _smc_spin_tick(self):
        self._smc_elapsed += 1.0 / 20.0

        if self._smc_elapsed >= self._smc_spin_dur:
            self._smc_timer.cancel()
            self._smc_timer = None
            self._cmd_vel_pub.publish(Twist())
            self.get_logger().info('[SMC] หมุนครบ → settle...')
            self._smc_start_settle()
            return

        twist = Twist()
        twist.angular.z = self._smc_spin_dir * SPIN_ANG_VEL
        self._cmd_vel_pub.publish(twist)

    def _cancel_smc(self):
        if self._smc_timer is not None:
            self._smc_timer.cancel()
            self._smc_timer = None
        self._smc_phase = 'idle'
        self._cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavNode()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()