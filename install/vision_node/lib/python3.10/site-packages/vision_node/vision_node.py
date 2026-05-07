import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from std_msgs.msg import Int32


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        # CAMERA
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Camera non détectée")
        else:
            self.get_logger().info("Camera OK")

        # PUBLISHERS
        self.servo1_pub = self.create_publisher(
            Int32,
            '/servo_s1',
            10
        )

        self.servo2_pub = self.create_publisher(
            Int32,
            '/servo_s2',
            10
        )

        self.hole_pub = self.create_publisher(
            Int32,
            '/hole_detected',
            10
        )

        # POSITION CAMERA
        self.servo1_pos = 90
        self.servo2_pos = 90

        # LIMITES
        self.MIN_ANGLE = 60
        self.MAX_ANGLE = 120

        # DIRECTION SCAN
        self.direction = 1

        # TIMER PLUS LENT
        self.timer = self.create_timer(
            0.2,
            self.process
        )

        self.get_logger().info("VISION NODE READY")

    def publish_servo1(self):

        msg = Int32()
        msg.data = int(self.servo1_pos)

        self.servo1_pub.publish(msg)

    def move_servo_scan(self):

        # scan plus lent
        self.servo1_pos += 1 * self.direction

        if self.servo1_pos >= 120:
            self.direction = -1

        elif self.servo1_pos <= 60:
            self.direction = 1

        self.publish_servo1()

    def process(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Image non reçue")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        gray = cv2.GaussianBlur(
            gray,
            (9, 9),
            2
        )

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.5,
            minDist=100,
            param1=100,
            param2=50,
            minRadius=20,
            maxRadius=60
        )

        detected = False

        if circles is not None:

            circles = np.uint16(np.around(circles))

            for c in circles[0]:

                x, y, r = c

                # zone detection
                if 200 < x < 500 and 100 < y < 400:

                    detected = True

                    cv2.circle(
                        frame,
                        (x, y),
                        r,
                        (0, 255, 0),
                        2
                    )

                    # =========================
                    # ZONE MORTE
                    # =========================

                    if x < 280:
                        self.servo1_pos -= 2

                    elif x > 360:
                        self.servo1_pos += 2

                    # limites
                    self.servo1_pos = max(
                        self.MIN_ANGLE,
                        min(self.MAX_ANGLE, self.servo1_pos)
                    )

                    self.publish_servo1()

                    # publication trou détecté
                    hole_msg = Int32()
                    hole_msg.data = 1

                    self.hole_pub.publish(hole_msg)

                    self.get_logger().info(
                        f"TROU TRACKÉ | X={x} | S1={self.servo1_pos}"
                    )

        # scan auto si rien détecté
        if not detected:

            hole_msg = Int32()
            hole_msg.data = 0

            self.hole_pub.publish(hole_msg)

            self.move_servo_scan()

        cv2.imshow("Camera", frame)

        cv2.waitKey(1)


def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()