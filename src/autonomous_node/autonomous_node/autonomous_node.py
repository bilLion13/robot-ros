import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AutonomousNode(Node):
    def __init__(self):
        super().__init__('autonomous_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.last_turn_time = 0

    def send_cmd(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def callback(self, msg):
        ranges = list(msg.ranges)
        front_ranges = ranges[0:15] + ranges[-15:]
        front_ranges = [r for r in front_ranges if 0.1 < r < 3.0]

        if not front_ranges:
            self.send_cmd(0.0, 0.0)
            return

        front_min = min(front_ranges)
        now = time.time()

        self.get_logger().info(f"FRONT DIST = {front_min:.2f}")

        if front_min < 0.35:
            self.send_cmd(0.0, 0.0)
            time.sleep(0.2)
            self.send_cmd(0.0, 0.35)
            self.last_turn_time = now
            self.get_logger().warn("Obstacle -> tourne un peu")
        else:
            self.send_cmd(0.12, 0.0)
            self.get_logger().info("Libre -> avance")


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_cmd(0.0, 0.0)
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()