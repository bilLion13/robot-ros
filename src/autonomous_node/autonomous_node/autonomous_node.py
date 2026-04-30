import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AutonomousNode(Node):
    def __init__(self):
        super().__init__('autonomous_node')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            10
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def callback(self, msg):
        # Nettoyage des valeurs
        ranges = [r for r in msg.ranges if r > 0.1]

        if not ranges:
            return

        min_dist = min(ranges)

        twist = Twist()

        # Log debug
        self.get_logger().info(f"MIN DIST = {min_dist:.2f}")

        if min_dist < 0.5:
            twist.angular.z = 0.3
            self.get_logger().warn(f"Obstacle ({min_dist:.2f}) → tourne")
        else:
            twist.linear.x = 0.2
            self.get_logger().info(f"Libre ({min_dist:.2f}) → avance")

        self.pub.publish(twist)


def main():
    rclpy.init()
    node = AutonomousNode()

    # STOP propre (pas obligatoire mais safe)
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    node.pub.publish(twist)

    node.destroy_node()
    rclpy.shutdown()