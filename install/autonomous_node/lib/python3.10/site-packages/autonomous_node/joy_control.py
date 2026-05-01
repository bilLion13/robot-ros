import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')

        self.create_subscription(Joy, '/joy', self.callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo1_pub = self.create_publisher(Int32, '/servo_s1', 10)
        self.servo2_pub = self.create_publisher(Int32, '/servo_s2', 10)

        self.s1 = 90
        self.s2 = 90

        self.get_logger().info('Joy control started')

    def callback(self, joy):
        twist = Twist()

        twist.linear.x = 0.20 * joy.axes[1]
        twist.angular.z = 0.80 * joy.axes[0]

        self.cmd_pub.publish(twist)

        if len(joy.axes) > 4:
            if joy.axes[3] > 0.5:
                self.s1 += 2
            elif joy.axes[3] < -0.5:
                self.s1 -= 2

            if joy.axes[4] > 0.5:
                self.s2 += 2
            elif joy.axes[4] < -0.5:
                self.s2 -= 2

        self.s1 = max(0, min(180, self.s1))
        self.s2 = max(0, min(180, self.s2))

        msg1 = Int32()
        msg2 = Int32()
        msg1.data = self.s1
        msg2.data = self.s2

        self.servo1_pub.publish(msg1)
        self.servo2_pub.publish(msg2)

    def stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = JoyControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()