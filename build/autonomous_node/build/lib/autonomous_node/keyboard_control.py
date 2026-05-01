import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo1_pub = self.create_publisher(Int32, '/servo_s1', 10)
        self.servo2_pub = self.create_publisher(Int32, '/servo_s2', 10)

        self.s1 = 90
        self.s2 = 90

        print("Controle clavier actif")
        print("Robot : z=avance, s=recule, q=gauche, d=droite, espace=stop")
        print("Camera : i=haut, k=bas, j=gauche, l=droite, c=centre")
        print("x=quitter")

    def publish_robot(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    def publish_camera(self):
        self.s1 = max(0, min(180, self.s1))
        self.s2 = max(0, min(180, self.s2))

        msg1 = Int32()
        msg2 = Int32()
        msg1.data = self.s1
        msg2.data = self.s2

        self.servo1_pub.publish(msg1)
        self.servo2_pub.publish(msg2)

        print(f"Camera s1={self.s1}, s2={self.s2}")


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()

    try:
        while True:
            key = get_key()

            if key == 'z':
                node.publish_robot(0.15, 0.0)
            elif key == 's':
                node.publish_robot(-0.15, 0.0)
            elif key == 'q':
                node.publish_robot(0.0, 0.5)
            elif key == 'd':
                node.publish_robot(0.0, -0.5)
            elif key == ' ':
                node.publish_robot(0.0, 0.0)
            elif key == 'j':
                node.s1 -= 5
                node.publish_camera()
            elif key == 'l':
                node.s1 += 5
                node.publish_camera()
            elif key == 'i':
                node.s2 += 5
                node.publish_camera()
            elif key == 'k':
                node.s2 -= 5
                node.publish_camera()
            elif key == 'c':
                node.s1 = 90
                node.s2 = 90
                node.publish_camera()
            elif key == 'x':
                break

            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass
    finally:
        node.publish_robot(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()