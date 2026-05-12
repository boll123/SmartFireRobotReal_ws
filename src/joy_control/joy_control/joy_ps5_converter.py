#!/usr/bin/env python3
"""
joy_ps5_converter.py  —  v3
─────────────────────────────────────────────
L2  (axis 2) = เดินหน้า  (analog proportional)
R2  (axis 5) = ถอยหลัง   (analog proportional)
L1  (btn  4) = เลี้ยวซ้าย (กด=MAX ทันที, ปล่อย=0)
R1  (btn  5) = เลี้ยวขวา  (กด=MAX ทันที, ปล่อย=0)
X   (btn  0) = Shutdown (ส่ง SIGINT ไปยัง process กลุ่ม → เหมือนกด Ctrl+C)
"""
import os
import signal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

LINEAR_SPEED  = 0.3   # m/s
ANGULAR_SPEED = 1.0   # rad/s

class JoyPS5Converter(Node):
    def __init__(self):
        super().__init__('joy_ps5_converter')
        self.sub = self.create_subscription(Joy, '/joy', self.joy_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_joy', 10)
        self._x_prev = 0  # debounce: กัน trigger ซ้ำ
        self.get_logger().info('joy_ps5_converter v3 ready')
        self.get_logger().info('  L2=forward  R2=reverse  L1=turn-left  R1=turn-right  X=shutdown')

    def joy_cb(self, msg: Joy):
        twist = Twist()

        # ── Linear: L2/R2 analog ─────────────────────────────────────────────
        l2 = (-msg.axes[2] + 1.0) / 2.0
        r2 = (-msg.axes[5] + 1.0) / 2.0
        twist.linear.x = (l2 - r2) * LINEAR_SPEED

        # ── Angular: L1/R1 digital ───────────────────────────────────────────
        l1 = int(msg.buttons[4])
        r1 = int(msg.buttons[5])
        if l1 and not r1:
            twist.angular.z =  ANGULAR_SPEED
        elif r1 and not l1:
            twist.angular.z = -ANGULAR_SPEED
        else:
            twist.angular.z =  0.0

        self.pub.publish(twist)

        # ── X button: Shutdown (Ctrl+C) ──────────────────────────────────────
        x_now = int(msg.buttons[0])
        if x_now == 1 and self._x_prev == 0:   # rising edge เท่านั้น
            self.get_logger().warn('X pressed → SIGINT (Ctrl+C)')
            os.killpg(os.getpgid(os.getpid()), signal.SIGINT)  # ส่งไปทั้ง process group
        self._x_prev = x_now


def main(args=None):
    rclpy.init(args=args)
    node = JoyPS5Converter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()