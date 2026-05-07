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

        self.mode = "AVANCE"
        self.turn_until = 0.0
        self.last_log_time = 0.0

        self.get_logger().info("Node autonome demarre")
        self.get_logger().info("Modes : AVANCE / STOP / TOURNE")

    def send_cmd(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub.publish(twist)

    def callback(self, msg):
        now = time.time()
        ranges = list(msg.ranges)

        front = ranges[0:20] + ranges[-20:]
        left = ranges[60:120]
        right = ranges[-120:-60]

        front = [r for r in front if 0.10 < r < 3.50]
        left = [r for r in left if 0.10 < r < 3.50]
        right = [r for r in right if 0.10 < r < 3.50]

        front_min = min(front) if front else 9.99
        left_min = min(left) if left else 9.99
        right_min = min(right) if right else 9.99

        if now - self.last_log_time > 0.5:
            self.get_logger().info(
                f"MODE={self.mode} | FRONT={front_min:.2f} | LEFT={left_min:.2f} | RIGHT={right_min:.2f}"
            )
            self.last_log_time = now

        if self.mode == "TOURNE":
            if now < self.turn_until:
                if left_min > right_min:
                    self.send_cmd(0.0, 0.45)
                else:
                    self.send_cmd(0.0, -0.45)
                return
            else:
                self.mode = "AVANCE"

        if front_min < 0.30:
            self.mode = "TOURNE"
            self.send_cmd(0.0, 0.0)
            self.turn_until = now + 1.2
            self.get_logger().warn("Obstacle proche -> STOP puis TOURNE")
            return

        if front_min < 0.55:
            self.mode = "AVANCE_LENT"
            self.send_cmd(0.07, 0.0)
            return

        self.mode = "AVANCE"
        self.send_cmd(0.15, 0.0)

    def stop_robot(self):
        self.get_logger().warn("Arret du robot")
        for _ in range(10):
            self.send_cmd(0.0, 0.0)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()