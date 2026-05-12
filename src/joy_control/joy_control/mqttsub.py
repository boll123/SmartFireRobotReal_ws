import rclpy

import paho.mqtt.client as mqtt

import json

from rclpy.node import Node

from geometry_msgs.msg import Twist
 
class MqttToRosBridge(Node):

    def __init__(self):

        super().__init__("mqtt_to_ros_bridge")

        self.publisher_ = self.create_publisher(Twist, "cmd_vel_received", 10)
 
        self.mqtt_client = mqtt.Client()

        self.mqtt_client.username_pw_set("parichu", "1122")

        self.mqtt_client.on_message = self.on_mqtt_message

        self.mqtt_client.connect("172.20.10.3", 1883)

        self.mqtt_client.subscribe("cmd_vel")

        self.mqtt_client.loop_start()

        self.get_logger().info("Bridge Started: Waiting for MQTT data...")
 
    def on_mqtt_message(self, client, userdata, msg):

        # รับค่าจาก MQTT แล้วแปลงกลับเป็น Dictionary

        data = json.loads(msg.payload.decode())

        # สร้าง ROS 2 Message

        ros_msg = Twist()

        ros_msg.linear.x = float(data['linear_speed'])

        # Publish ลงในเครื่องลูก

        self.publisher_.publish(ros_msg)

        self.get_logger().info(f"MQTT -> ROS2: {data['linear_speed']} m/s")
 
def main(args=None):

    rclpy.init(args=args)

    node = MqttToRosBridge()

    rclpy.spin(node)

    rclpy.shutdown()
 
if __name__ == "__main__":

    main()
 