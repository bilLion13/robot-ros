import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import Int32


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Camera non détectée")
        else:
            self.get_logger().info("Camera OK")

        # servo
        self.servo_pub = self.create_publisher(Int32, '/servo_s1', 10)

        self.angle = 60
        self.direction = 1

        # timers
        self.timer_cam = self.create_timer(0.1, self.process)
        self.timer_servo = self.create_timer(2.0, self.move_servo)

    def move_servo(self):
        self.angle += 20 * self.direction

        if self.angle >= 120:
            self.direction = -1
        elif self.angle <= 60:
            self.direction = 1

        msg = Int32()
        msg.data = self.angle
        self.servo_pub.publish(msg)

        self.get_logger().info(f"Servo angle: {self.angle}")

    def process(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Image non reçue")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

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

        if circles is not None:
            circles = np.uint16(np.around(circles))

            for c in circles[0]:
                x, y, r = c

                # filtre position (ex: centre image seulement)
                if 200 < x < 500 and 100 < y < 400:
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                    self.get_logger().warn("TROU VALIDÉ")

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