import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class JoyControl(Node):

    def __init__(self):

        super().__init__('joy_control')

        # SUBSCRIBER
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.callback,
            10
        )

        # PUBLISHERS
        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel',10)

        self.servo1_pub = self.create_publisher(Int32,'/servo_s1',10 )

        self.servo2_pub = self.create_publisher( Int32,'/servo_s2',10)

        # POSITIONS CAMERA
        self.servo1 = 90
        self.servo2 = 90

        # LIMITES
        self.MIN_ANGLE = 0
        self.MAX_ANGLE = 180

        # MODE
        self.mode = "manual"

        # POSITION INITIALE
        self.publish_servos()

        self.get_logger().info("JOYSTICK CONTROL READY")

    # ==========================================
    # PUBLICATION SERVOS
    # ==========================================

    def publish_servos(self):

        s1 = Int32()
        s1.data = int(self.servo1)

        s2 = Int32()
        s2.data = int(self.servo2)

        self.servo1_pub.publish(s1)
        self.servo2_pub.publish(s2)

    # ==========================================
    # CALLBACK JOYSTICK
    # ==========================================

    def callback(self, msg):

        twist = Twist()

        # ==========================================
        # CHANGEMENT MODE
        # L1 = manuel
        # R1 = auto
        # ==========================================

        # L1
        if msg.buttons[6] == 1:

            self.mode = "manual"

            self.get_logger().info("MODE MANUEL")

        # R1
        if msg.buttons[7] == 1:

            self.mode = "auto"

            self.get_logger().info("MODE AUTO")

        # ==========================================
        # MODE MANUEL
        # ==========================================

        if self.mode == "manual":

            # ==========================================
            # STICK GAUCHE = ROBOT
            # ==========================================

            linear = msg.axes[1]
            angular = msg.axes[0]

            # DEADZONE
            if abs(linear) < 0.1:
                linear = 0.0

            if abs(angular) < 0.1:
                angular = 0.0

            # ROBOT
            twist.linear.x = 0.3 * linear
            twist.angular.z = 1.0 * angular

            self.cmd_pub.publish(twist)

            # ==========================================
            # CAMERA AVEC BOUTONS
            # ==========================================

            # A = bas
            if msg.buttons[0] == 1:

                self.servo1 -= 2

            # B = droite
            if msg.buttons[1] == 1:

                self.servo1 += 2

            # X = gauche
            if msg.buttons[2] == 1:

                self.servo2 += 2

            # Y = haut
            if msg.buttons[4] == 1:

                self.servo2 -= 2

            # LIMITES
            self.servo1 = max(
                self.MIN_ANGLE,
                min(self.MAX_ANGLE, self.servo1)
            )

            self.servo2 = max(
                self.MIN_ANGLE,
                min(self.MAX_ANGLE, self.servo2)
            )

            # ENVOI
            self.publish_servos()

            # DEBUG
            self.get_logger().info(
                f"S1={self.servo1} | S2={self.servo2}"
            )


def main():

    rclpy.init()
    node = JoyControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()