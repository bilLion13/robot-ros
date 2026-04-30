import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# Influx
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        #  Connexion Influx
        self.client = InfluxDBClient(
            url="https://us-east-1-1.aws.cloud2.influxdata.com",
            token="X69mccEPDdx-0j3GGWPjSgYaKcJHLs3NHTWAbO4Qpp970uvsfwQJLmYk-6ZHyuDGreekVBuwueXYnW3I6jzXyw==",
            org="Wb"
        )

        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)

        #  Subscription LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )

    def callback(self, msg):
        ranges = [r for r in msg.ranges if r > 0.0]

        if not ranges:
            return

        min_dist = min(ranges)

        # LOG
        self.get_logger().info(f"Distance min: {min_dist:.2f} m")

        #  Détection
        if min_dist < 0.5:
            self.get_logger().warn(" OBSTACLE PROCHE !")
            print("DEBUG LIDAR 222:", min_dist)

        #  ENVOI INFLUX
        try:
            point = Point("robot_lidar").field("distance", float(min_dist))

            self.write_api.write(
                bucket="robot-ros-pi",
                record=point
            )
            print("DEBUG LIDAR:", min_dist)

            self.get_logger().info(f"LIDAR envoyé: {min_dist:.2f}")

        except Exception as e:
            self.get_logger().error(f"Erreur Influx: {e}")


def main():
    rclpy.init()
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()