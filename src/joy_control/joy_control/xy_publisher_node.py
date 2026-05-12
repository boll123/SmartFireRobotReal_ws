#!/usr/bin/env python3
# xy_publisher_node.py  —  v3-UseQuaternionMsg
# ─────────────────────────────────────────────────────────────────
# Node: xy_publisher_node
# ส่ง XY + yaw goal ผ่าน topic /goal_xy (geometry_msgs/Quaternion)
#
# สิ่งที่แก้ไขจาก v2:
#   - เปลี่ยน message type: Point → Quaternion
#   - w = องศา (ใส่ตรงๆ ไม่ต้องแปลง radian เอง)
#
# field ที่ใช้:
#   x = X ในแผนที่ (map frame)
#   y = Y ในแผนที่ (map frame)
#   z = 0.0 (ไม่ใช้)
#   w = ทิศหันหน้า (องศา: 0=ตรง, 90=ซ้าย, 180=หลัง, -90=ขวา)
#
# Usage (interactive):
#   ros2 run joy_control xy_publisher_node
#   → พิมพ์: 5.0 2.0          ไปที่ (5, 2) หัวตรง
#   → พิมพ์: 5.0 2.0 90       ไปที่ (5, 2) หัน 90°
#   → พิมพ์: 0 0              กลับจุด spawn
#   → พิมพ์: q                ออก
#
# Usage (topic ตรง):
#   ros2 topic pub --once /goal_xy geometry_msgs/msg/Quaternion "{x: 5.0, y: 2.0, z: 0.0, w: 90.0}"
# ─────────────────────────────────────────────────────────────────

import math
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Quaternion


class XYPublisherNode(Node):
    def __init__(self):
        super().__init__('xy_publisher_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('x',       float('nan'))
        self.declare_parameter('y',       float('nan'))
        self.declare_parameter('yaw_deg', 0.0)

        # ── Publisher ─────────────────────────────────────────────
        self._pub = self.create_publisher(Quaternion, '/goal_xy', 10)

        self.get_logger().info(
            '\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            ' [XYPublisherNode] พร้อมส่ง /goal_xy\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            ' พิมพ์:  x y [องศา]\n'
            ' ตัวอย่าง:\n'
            '   5.0 2.0          → ไปที่ (5.0, 2.0)  หัวตรง\n'
            '   5.0 2.0 90       → ไปที่ (5.0, 2.0)  หันซ้าย 90°\n'
            '   5.0 2.0 180      → ไปที่ (5.0, 2.0)  หันกลับ\n'
            '   5.0 2.0 -90      → ไปที่ (5.0, 2.0)  หันขวา\n'
            '   0 0              → กลับจุด spawn\n'
            '   q                → ออก\n'
            ' origin (0,0) = จุด spawn ของหุ่นใน Gazebo\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )

        # ── ตรวจว่ามี parameter ส่งมาหรือเปล่า ──────────────────
        x_param = self.get_parameter('x').value
        y_param = self.get_parameter('y').value
        if not math.isnan(x_param) and not math.isnan(y_param):
            yaw_deg = self.get_parameter('yaw_deg').value
            self.get_logger().info(
                f'[XYPublisherNode] ส่งจาก parameter: '
                f'x={x_param}  y={y_param}  yaw={yaw_deg}°'
            )
            self._publish(x_param, y_param, yaw_deg)

        # ── interactive CLI thread ────────────────────────────────
        self._cli_thread = threading.Thread(
            target=self._cli_loop, daemon=True
        )
        self._cli_thread.start()

    # ──────────────────────────────────────────────────────────────
    def _publish(self, x: float, y: float, yaw_deg: float = 0.0):
        msg = Quaternion()
        msg.x = x
        msg.y = y
        msg.z = 0.0
        msg.w = yaw_deg   # w = องศา
        self._pub.publish(msg)
        self.get_logger().info(
            f'[XYPublisherNode] ส่ง goal → '
            f'x={x:.3f}  y={y:.3f}  yaw={yaw_deg:.1f}°'
        )

    # ──────────────────────────────────────────────────────────────
    def _cli_loop(self):
        while rclpy.ok():
            try:
                raw = input('\n[goal_xy] พิมพ์ "x y [องศา]" หรือ "q" เพื่อออก: ').strip()
            except (EOFError, KeyboardInterrupt, OSError):
                break

            if raw.lower() in ('q', 'quit', 'exit'):
                self.get_logger().info('[XYPublisherNode] ออกจากโหมด CLI')
                break

            if not raw:
                continue

            parts = raw.split()
            if len(parts) < 2:
                print('  [!] ต้องระบุอย่างน้อย 2 ค่า: x y')
                continue

            try:
                x       = float(parts[0])
                y       = float(parts[1])
                yaw_deg = float(parts[2]) if len(parts) >= 3 else 0.0
            except ValueError:
                print('  [!] ค่าไม่ถูกต้อง ต้องเป็นตัวเลข เช่น: 5.0 2.0 90')
                continue

            self._publish(x, y, yaw_deg)


# ─────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = XYPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()