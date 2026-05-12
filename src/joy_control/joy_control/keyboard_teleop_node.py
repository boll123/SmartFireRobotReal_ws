#!/usr/bin/env python3
# keyboard_teleop_node.py  —  v1
# ─────────────────────────────────────────────────────────────────
# กดค้าง = หุ่นเดิน, ปล่อย = หยุดทันที
#
# หลักการ:
#   - ใช้ tty raw mode + select() timeout=0.08s
#   - ถ้ามี key input ภายใน timeout → set velocity + active=True
#   - ถ้า timeout (ไม่มี key) → set velocity=0 + active=False
#   - timer publish @ 20Hz ตลอดเวลา
#   - ZeroLatch ใน esp32_hardware_interface v16 จะรับ cmd=0
#     แล้ว force stop ทันที ไม่รอ decel ramp
#
# Key bindings:
#   w = forward       s = backward
#   a = turn left     d = turn right
#   q = forward-left  e = forward-right
#   z = back-left     x = back-right
#   Ctrl+C = quit
#
# วางไฟล์นี้ที่: joy_control/scripts/keyboard_teleop_node.py
# ─────────────────────────────────────────────────────────────────

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import threading

# ── Velocity parameters ───────────────────────────────────────────
LINEAR_VEL   = 0.5    # m/s  ปรับได้ตามต้องการ
ANGULAR_VEL  = 1.2    # rad/s
PUBLISH_RATE = 20.0   # Hz

# ── Key timeout ───────────────────────────────────────────────────
# ถ้าไม่มี keypress ใน 0.08s → ถือว่าปล่อยคีย์แล้ว
# ค่าต่ำ = response ไว แต่ถ้า CPU load สูงอาจเกิด false stop
# แนะนำ 0.08 - 0.12
KEY_TIMEOUT = 0.08

# ── Key bindings: key → (linear.x, angular.z) ────────────────────
KEY_BINDINGS = {
    'w': ( LINEAR_VEL,           0.0),   # forward
    's': (-LINEAR_VEL,           0.0),   # backward
    'a': ( 0.0,          ANGULAR_VEL),   # turn left
    'd': ( 0.0,         -ANGULAR_VEL),   # turn right
    'q': ( LINEAR_VEL,   ANGULAR_VEL),   # forward-left diagonal
    'e': ( LINEAR_VEL,  -ANGULAR_VEL),   # forward-right diagonal
    'z': (-LINEAR_VEL,   ANGULAR_VEL),   # backward-left diagonal
    'x': (-LINEAR_VEL,  -ANGULAR_VEL),   # backward-right diagonal
}

HELP_MSG = """
┌─────────────────────────────────────────┐
│  Keyboard Teleop  (กดค้าง=เดิน, ปล่อย=หยุด)  │
├─────────────────────────────────────────┤
│    q    w    e                          │
│    a    s    d                          │
│    z    x                               │
│                                         │
│  w/s  = forward / backward              │
│  a/d  = turn left / right               │
│  q/e  = diagonal forward                │
│  z/x  = diagonal backward               │
│  Ctrl+C = quit                          │
└─────────────────────────────────────────┘
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Declare parameters (ปรับได้จาก launch หรือ yaml)
        self.declare_parameter('linear_vel',   LINEAR_VEL)
        self.declare_parameter('angular_vel',  ANGULAR_VEL)
        self.declare_parameter('publish_rate', PUBLISH_RATE)
        self.declare_parameter('key_timeout',  KEY_TIMEOUT)
        self.declare_parameter('cmd_topic',    '/cmd_vel_keyboard')

        lin   = self.get_parameter('linear_vel').value
        ang   = self.get_parameter('angular_vel').value
        rate  = self.get_parameter('publish_rate').value
        self._key_timeout = self.get_parameter('key_timeout').value
        topic = self.get_parameter('cmd_topic').value

        # rebuild bindings with actual param values
        scale = lin / LINEAR_VEL if LINEAR_VEL != 0 else 1.0
        ang_s = ang / ANGULAR_VEL if ANGULAR_VEL != 0 else 1.0
        self._bindings = {
            k: (v[0] * scale, v[1] * ang_s)
            for k, v in KEY_BINDINGS.items()
        }

        self._pub = self.create_publisher(Twist, topic, 10)

        self._lin    = 0.0
        self._ang    = 0.0
        self._active = False
        self._lock   = threading.Lock()
        self._running = True

        # publish timer
        self.create_timer(1.0 / rate, self._publish_cb)

        # keyboard read thread (daemon → จะ terminate เมื่อ main thread จบ)
        self._kb_thread = threading.Thread(
            target=self._read_keys, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(HELP_MSG)
        self.get_logger().info(
            f'Publishing to: {topic} @ {rate}Hz  '
            f'lin={lin}m/s  ang={ang}rad/s  timeout={self._key_timeout}s')

    # ── publish callback (timer) ──────────────────────────────────
    def _publish_cb(self):
        msg = Twist()
        with self._lock:
            if self._active:
                msg.linear.x  = self._lin
                msg.angular.z = self._ang
            else:
                msg.linear.x  = 0.0
                msg.angular.z = 0.0
        self._pub.publish(msg)

    # ── keyboard reader thread ────────────────────────────────────
    def _read_keys(self):
        fd  = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running and rclpy.ok():
                ready = select.select([sys.stdin], [], [], self._key_timeout)[0]

                if ready:
                    key = sys.stdin.read(1)

                    # Ctrl+C
                    if key == '\x03':
                        self.get_logger().info('Ctrl+C received — shutting down keyboard teleop')
                        self._running = False
                        rclpy.shutdown()
                        break

                    if key in self._bindings:
                        lin, ang = self._bindings[key]
                        with self._lock:
                            self._lin    = lin
                            self._ang    = ang
                            self._active = True
                    else:
                        # กดคีย์อื่นที่ไม่ใช่ movement key → stop
                        with self._lock:
                            self._lin    = 0.0
                            self._ang    = 0.0
                            self._active = False

                else:
                    # select timeout → ไม่มีการกดคีย์ → หยุด
                    with self._lock:
                        self._lin    = 0.0
                        self._ang    = 0.0
                        self._active = False

        except Exception as e:
            self.get_logger().error(f'Keyboard read error: {e}')
        finally:
            # restore terminal settings
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
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