#!/usr/bin/env python3
"""
pose_saver_node.py  —  v1
─────────────────────────────────────────────────────────────────
Subscribe /amcl_pose แล้ว save ลงไฟล์ yaml อัตโนมัติทุก 5 วินาที
เมื่อ pose stable (covariance ต่ำ)

Save path: ~/SmartFireRobotReal_ws/src/joy_control/maps/last_pose.yaml

Format:
  x: 1.234
  y: -0.567
  yaw: 0.785

ใช้งาน:
  - Node นี้รันใน nav launch อัตโนมัติ
  - ทุกครั้งที่ AMCL localize ได้ดี → save pose ใหม่ทับ
  - ครั้งต่อไปที่ launch → อ่านค่านี้เป็น initial_pose
─────────────────────────────────────────────────────────────────
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import yaml
import math
import os
import time


# path สำหรับ save pose
POSE_FILE = os.path.expanduser(
    '~/SmartFireRobotReal_ws/src/joy_control/maps/last_pose.yaml'
)

# covariance threshold — save เมื่อ pose stable
# ค่า cov[0]=xx, cov[7]=yy, cov[35]=yaw_yaw
COV_XY_THRESH  = 0.05   # m² — ถ้า uncertainty x,y น้อยกว่านี้ถือว่า stable
COV_YAW_THRESH = 0.05   # rad²

# save ทุกกี่วินาที (ป้องกัน write ถี่เกิน)
SAVE_INTERVAL_SEC = 5.0


class PoseSaverNode(Node):
    def __init__(self):
        super().__init__('pose_saver_node')

        self.last_save_time = 0.0
        self.save_count     = 0

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_cb,
            10
        )

        self.get_logger().info(
            f'PoseSaverNode ready — saving to: {POSE_FILE}'
        )
        self.get_logger().info(
            f'  cov_xy_thresh={COV_XY_THRESH}  '
            f'cov_yaw_thresh={COV_YAW_THRESH}  '
            f'interval={SAVE_INTERVAL_SEC}s'
        )

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance  # 6x6 row-major

        cov_xx  = cov[0]   # index 0
        cov_yy  = cov[7]   # index 7
        cov_yaw = cov[35]  # index 35

        # เช็ค stability
        if cov_xx > COV_XY_THRESH or cov_yy > COV_XY_THRESH:
            return  # pose ยังไม่ stable พอ
        if cov_yaw > COV_YAW_THRESH:
            return

        # rate limit
        now = time.time()
        if now - self.last_save_time < SAVE_INTERVAL_SEC:
            return
        self.last_save_time = now

        # แปลง quaternion → yaw
        q = msg.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # save ลงไฟล์
        data = {
            'x':   round(x,   4),
            'y':   round(y,   4),
            'yaw': round(yaw, 4),
        }

        try:
            os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
            with open(POSE_FILE, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

            self.save_count += 1
            self.get_logger().info(
                f'[#{self.save_count}] Pose saved: '
                f'x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}°  '
                f'(cov_xx={cov_xx:.4f} cov_yy={cov_yy:.4f} '
                f'cov_yaw={cov_yaw:.4f})'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to save pose: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()