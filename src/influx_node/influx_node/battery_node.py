import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from std_msgs.msg import Int32
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')

        self.client = InfluxDBClient(
            url="https://us-east-1-1.aws.cloud2.influxdata.com",
            token="X69mccEPDdx-0j3GGWPjSgYaKcJHLs3NHTWAbO4Qpp970uvsfwQJLmYk-6ZHyuDGreekVBuwueXYnW3I6jzXyw==",
            org="Wb"
        )

        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)

        self.sub = self.create_subscription(
            UInt16,
            '/battery',
            self.callback,
            10
        )
        self.hole_sub = self.create_subscription(
            Int32,
            '/hole_detected',
            self.hole_callback,
            10
        )

    def callback(self, msg):
        value = msg.data

        print("DEBUG BATTERY:", value)

        try:
            point = Point("robot_battery").field("value", value)
            self.write_api.write(bucket="robot-ros-pi", record=point)

            self.get_logger().info(f"Batterie envoyée: {value}")

        except Exception as e:
            self.get_logger().error(f"Erreur Influx: {e}")

    def hole_callback(self, msg):
        value = msg.data

        try:
            point = Point("robot_hole").field("detected", value)
            self.write_api.write(bucket="robot-ros-pi", record=point)

            self.get_logger().info(f"Trou envoyé: {value}")

        except Exception as e:
            self.get_logger().error(f"Erreur Influx trou: {e}")

def main():
    rclpy.init()
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()


