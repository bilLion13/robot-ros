import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
import subprocess
import os
import signal


class JoyControl(Node):

    def __init__(self):
        super().__init__('joy_control')

        self.sub = self.create_subscription(Joy, '/joy', self.callback, 10)

        self.cmd_pub      = self.create_publisher(Twist, '/cmd_vel',  10)
        self.servo1_pub   = self.create_publisher(Int32, '/servo_s1', 10)
        self.servo2_pub   = self.create_publisher(Int32, '/servo_s2', 10)
        self.joystate_pub = self.create_publisher(Bool,  '/JoyState', 10)

        # ==========================================
        # CAMERA (valeurs Yahboom réelles)
        # servo_s1 = horizontal : -90 à +90,  init  0
        # servo_s2 = vertical   : -90 à +20,  init -60
        # ==========================================
        self.servo1 = 0    # horizontal
        self.servo2 = -60  # vertical

        self.S1_MIN, self.S1_MAX = -90, 90
        self.S2_MIN, self.S2_MAX = -90, 20

        self.mode = "manual"
        self.auto_process = None

        self.publish_servos()
        self.get_logger().info("JOYSTICK CONTROL READY")
        self.get_logger().info(f"servo1(H)={self.servo1} | servo2(V)={self.servo2}")

    def publish_servos(self):
        s1 = Int32()
        s1.data = int(self.servo1)
        s2 = Int32()
        s2.data = int(self.servo2)
        self.servo1_pub.publish(s1)
        self.servo2_pub.publish(s2)

    def start_auto_node(self):
        if self.auto_process is not None:
            self.get_logger().info("Node autonome déjà actif")
            return
        try:
            self.auto_process = subprocess.Popen(
                ['ros2', 'run', 'autonomous_node', 'autonomous_node'],
                env={**os.environ, 'ROS_DOMAIN_ID': '20'}
            )
            self.get_logger().info(f"Node autonome démarré PID={self.auto_process.pid}")
        except Exception as e:
            self.get_logger().error(f"Erreur démarrage : {e}")

    def stop_auto_node(self):
        if self.auto_process is None:
            return
        try:
            self.auto_process.send_signal(signal.SIGINT)
            self.auto_process.wait(timeout=3)
        except Exception:
            self.auto_process.kill()
        self.auto_process = None
        self.cmd_pub.publish(Twist())
        self.get_logger().info("Node autonome arrêté")

    def set_joystate(self, active: bool):
        msg = Bool()
        msg.data = active
        for _ in range(3):
            self.joystate_pub.publish(msg)

    def callback(self, msg):

        # ==========================================
        # MODES  — LT[6] = Manuel  |  RT[7] = Auto
        # ==========================================
        if msg.buttons[6] == 1 and self.mode != "manual":
            self.mode = "manual"
            self.stop_auto_node()
            self.set_joystate(False)
            self.get_logger().info("◀  MODE MANUEL")

        if msg.buttons[7] == 1 and self.mode != "auto":
            self.mode = "auto"
            self.start_auto_node()
            self.set_joystate(True)
            self.cmd_pub.publish(Twist())
            self.get_logger().info("▶  MODE AUTO")

        if self.mode == "manual":

            # ── ROBOT — stick gauche ──
            linear  = msg.axes[1]
            angular = msg.axes[0]

            if abs(linear)  < 0.1: linear  = 0.0
            if abs(angular) < 0.1: angular = 0.0

            twist = Twist()
            twist.linear.x  = 0.3 * linear
            twist.angular.z = 1.0 * angular
            self.cmd_pub.publish(twist)

            # ── CAMÉRA HORIZONTALE → servo_s1 ──
            # X[2] = gauche  (servo1 diminue  → tourne à gauche)
            # B[1] = droite  (servo1 augmente → tourne à droite)
            if msg.buttons[2] == 1:
                self.servo1 -= 2   # gauche
            if msg.buttons[1] == 1:
                self.servo1 += 2   # droite

            # ── CAMÉRA VERTICALE → servo_s2 ──
            # Y[3] = haut  (servo2 augmente → lève,   max +20)
            # A[0] = bas   (servo2 diminue  → baisse, min -90)
            if msg.buttons[3] == 1:
                self.servo2 += 2   # lève
            if msg.buttons[0] == 1:
                self.servo2 -= 2   # baisse

            self.servo1 = max(self.S1_MIN, min(self.S1_MAX, self.servo1))
            self.servo2 = max(self.S2_MIN, min(self.S2_MAX, self.servo2))

            self.publish_servos()

            self.get_logger().info(
                f"H={self.servo1:4d}°  V={self.servo2:4d}°  "
                f"lin={twist.linear.x:.2f}  ang={twist.angular.z:.2f}"
            )

    def destroy_node(self):
        self.stop_auto_node()
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main():
    rclpy.init()
    node = JoyControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()