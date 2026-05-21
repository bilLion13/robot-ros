import time
import math
import json

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Int32


# ============================================================
# PARAMÈTRES DE VITESSE
# ============================================================

# Vitesse normale
LINEAR_FAST = 0.06

# Vitesse lente proche obstacle
LINEAR_SLOW = 0.03

# Recul obstacle lidar
LINEAR_BACK = -0.05

# Rotation normale lidar
ANGULAR_TURN = 0.45


# ============================================================
# DISTANCES DE SÉCURITÉ
# ============================================================

DIST_EMERGENCY = 0.18
DIST_STOP = 0.42
DIST_SLOW = 0.75
DIST_CLEAR = 0.65
DIST_SIDE = 0.25


# ============================================================
# TEMPS
# ============================================================

BACK_DURATION = 0.7
MAX_TURN_DURATION = 2.5
LOG_INTERVAL = 0.4


# ============================================================
# CAMÉRA
# ============================================================

CAMERA_VERTICAL_ANGLE = -45


class AutonomousNode(Node):

    def __init__(self):
        super().__init__('autonomous_node')

        # ====================================================
        # PUBLISHERS
        # ====================================================

        self.pub_cmd = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.pub_diag = self.create_publisher(
            String,
            '/robot/diagnostics',
            10
        )

        self.servo2_pub = self.create_publisher(
            Int32,
            '/servo_s2',
            10
        )

        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.sub_joy = self.create_subscription(
            Bool,
            '/JoyState',
            self.joystate_callback,
            10
        )

        self.sub_danger = self.create_subscription(
            Int32,
            '/danger_detected',
            self.danger_callback,
            10
        )

        # ====================================================
        # VARIABLES
        # ====================================================

        self.active = True

        self.mode = "INIT"

        self.front_min = 9.99
        self.left_min = 9.99
        self.right_min = 9.99

        self.turn_dir = 1

        self.state_until = 0.0
        self.danger_until = 0.0

        # Ignore temporairement les nouvelles détections
        self.ignore_danger_until = 0.0

        self.danger_detected = False

        self.last_log = 0.0
        self.scan_count = 0

        # ====================================================
        # TIMER CAMÉRA
        # ====================================================

        self.camera_timer = self.create_timer(
            1.0,
            self.publish_camera_position
        )

        self.publish_camera_position()

        self.get_logger().info("=" * 60)
        self.get_logger().info("AUTONOMOUS NODE READY")
        self.get_logger().info("=" * 60)

    # ========================================================
    # POSITION CAMÉRA
    # ========================================================

    def publish_camera_position(self):

        msg = Int32()
        msg.data = int(CAMERA_VERTICAL_ANGLE)

        self.servo2_pub.publish(msg)

    # ========================================================
    # MODE MANUEL / AUTO
    # ========================================================

    def joystate_callback(self, msg: Bool):

        # JoyState=True  → manuel
        # JoyState=False → auto

        self.active = not msg.data

        if not self.active:

            self.mode = "MANUEL"

            self.send_cmd(0.0, 0.0)

            self.get_logger().info(
                "MODE MANUEL"
            )

        else:

            self.mode = "INIT"

            self.publish_camera_position()

            self.get_logger().info(
                "MODE AUTO"
            )

    # ========================================================
    # DÉTECTION DANGER CAMÉRA
    # ========================================================

    def danger_callback(self, msg: Int32):

        now = time.time()

        # Ignore temporairement les nouvelles détections
        if now < self.ignore_danger_until:
            return

        detected = (msg.data == 1)

        # ====================================================
        # DANGER DÉTECTÉ
        # ====================================================

        if detected and self.active:

            # Évite de relancer sans arrêt
            if self.mode in [
                "RECULE_DANGER",
                "TOURNE_DANGER"
            ]:
                return

            self.danger_detected = True

            self.get_logger().warn(
                "DANGER DETECTE"
            )

            # Stop rapide
            self.send_cmd(0.0, 0.0)

            # Petit recul uniquement
            self.mode = "RECULE_DANGER"

            # Recul très court
            self.danger_until = now + 0.25

            # Choisir direction
            self.choose_turn_direction()

        else:

            self.danger_detected = False

    # ========================================================
    # ENVOI COMMANDE MOTEUR
    # ========================================================

    def send_cmd(self, linear, angular):

        twist = Twist()

        twist.linear.x = float(linear)
        twist.angular.z = float(angular)

        self.pub_cmd.publish(twist)

    # ========================================================
    # FILTRAGE LIDAR
    # ========================================================

    @staticmethod
    def safe_min(values, lo=0.08, hi=3.5):

        valid = []

        for value in values:

            if math.isnan(value):
                continue

            if math.isinf(value):
                continue

            if lo < value < hi:
                valid.append(value)

        if len(valid) == 0:
            return 9.99

        return min(valid)

    # ========================================================
    # EXTRACTION SECTEURS
    # ========================================================

    def get_sector_values(
        self,
        msg,
        angle_min_deg,
        angle_max_deg
    ):

        values = []

        for i, distance in enumerate(msg.ranges):

            angle_rad = (
                msg.angle_min +
                i * msg.angle_increment
            )

            angle_deg = math.degrees(angle_rad)

            while angle_deg > 180:
                angle_deg -= 360

            while angle_deg < -180:
                angle_deg += 360

            if angle_min_deg <= angle_deg <= angle_max_deg:
                values.append(distance)

        return values

    # ========================================================
    # DÉCOUPAGE LIDAR
    # ========================================================

    def parse_scan(self, msg):

        front_values = self.get_sector_values(
            msg,
            -18,
            18
        )

        left_values = self.get_sector_values(
            msg,
            35,
            95
        )

        right_values = self.get_sector_values(
            msg,
            -95,
            -35
        )

        self.front_min = self.safe_min(front_values)
        self.left_min = self.safe_min(left_values)
        self.right_min = self.safe_min(right_values)

    # ========================================================
    # CALLBACK PRINCIPAL
    # ========================================================

    def lidar_callback(self, msg: LaserScan):

        if not self.active:
            return

        now = time.time()

        self.scan_count += 1

        self.parse_scan(msg)

        # ====================================================
        # RECUL DANGER
        # ====================================================

        if self.mode == "RECULE_DANGER":

            if now < self.danger_until:

                # Petit recul doux
                self.send_cmd(
                    -0.025,
                    0.08 * self.turn_dir
                )

                self.publish_diag(
                    -0.025,
                    0.08 * self.turn_dir
                )

                self.log_status(now)

                return

            # Passe au virage
            self.mode = "TOURNE_DANGER"

            self.state_until = now + 0.9

            return

        # ====================================================
        # VIRAGE DANGER
        # ====================================================

        if self.mode == "TOURNE_DANGER":

            if now > self.state_until:

                self.mode = "AVANCE"

                self.danger_detected = False

                # Ignore danger pendant 2 sec
                self.ignore_danger_until = (
                    time.time() + 2.0
                )

                self.send_cmd(0.0, 0.0)

                self.publish_diag(0.0, 0.0)

                self.log_status(now)

                return

            # Courbe réaliste
            self.send_cmd(
                0.035,
                0.9 * self.turn_dir
            )

            self.publish_diag(
                0.035,
                0.9 * self.turn_dir
            )

            self.log_status(now)

            return

        # ====================================================
        # RECUL LIDAR
        # ====================================================

        if self.mode == "RECULE":

            if now < self.state_until:

                self.send_cmd(
                    LINEAR_BACK,
                    0.0
                )

                self.publish_diag(
                    LINEAR_BACK,
                    0.0
                )

                self.log_status(now)

                return

            self.choose_turn_direction()

            self.mode = "TOURNE"

            self.state_until = (
                now + MAX_TURN_DURATION
            )

            return

        # ====================================================
        # VIRAGE LIDAR
        # ====================================================

        if self.mode == "TOURNE":

            if self.front_min > DIST_CLEAR:

                self.mode = "AVANCE"

                self.send_cmd(0.0, 0.0)

                self.publish_diag(0.0, 0.0)

                self.log_status(now)

                return

            if now > self.state_until:

                self.mode = "AVANCE"

                self.send_cmd(0.0, 0.0)

                self.publish_diag(0.0, 0.0)

                self.log_status(now)

                return

            self.send_cmd(
                0.0,
                ANGULAR_TURN * self.turn_dir
            )

            self.publish_diag(
                0.0,
                ANGULAR_TURN * self.turn_dir
            )

            self.log_status(now)

            return

        # ====================================================
        # OBSTACLE TRÈS PROCHE
        # ====================================================

        if self.front_min < DIST_EMERGENCY:

            self.mode = "RECULE"

            self.state_until = (
                now + BACK_DURATION
            )

            self.send_cmd(0.0, 0.0)

            self.publish_diag(0.0, 0.0)

            self.log_status(now)

            return

        # ====================================================
        # OBSTACLE DEVANT
        # ====================================================

        if self.front_min < DIST_STOP:

            self.choose_turn_direction()

            self.mode = "TOURNE"

            self.state_until = (
                now + MAX_TURN_DURATION
            )

            self.send_cmd(0.0, 0.0)

            self.publish_diag(0.0, 0.0)

            self.log_status(now)

            return

        # ====================================================
        # RALENTISSEMENT
        # ====================================================

        if self.front_min < DIST_SLOW:

            self.mode = "AVANCE_LENT"

            linear = LINEAR_SLOW
            angular = 0.0

            if self.left_min > self.right_min + 0.15:
                angular = 0.18

            elif self.right_min > self.left_min + 0.15:
                angular = -0.18

            self.send_cmd(linear, angular)

            self.publish_diag(
                linear,
                angular
            )

            self.log_status(now)

            return

        # ====================================================
        # CORRECTION CÔTÉS
        # ====================================================

        if self.left_min < DIST_SIDE:

            self.mode = "CORRIGE_DROITE"

            self.send_cmd(
                LINEAR_SLOW,
                -0.20
            )

            self.publish_diag(
                LINEAR_SLOW,
                -0.20
            )

            self.log_status(now)

            return

        if self.right_min < DIST_SIDE:

            self.mode = "CORRIGE_GAUCHE"

            self.send_cmd(
                LINEAR_SLOW,
                0.20
            )

            self.publish_diag(
                LINEAR_SLOW,
                0.20
            )

            self.log_status(now)

            return

        # ====================================================
        # ROUTE LIBRE
        # ====================================================

        self.mode = "AVANCE"

        self.send_cmd(
            LINEAR_FAST,
            0.0
        )

        self.publish_diag(
            LINEAR_FAST,
            0.0
        )

        self.log_status(now)

    # ========================================================
    # CHOIX DIRECTION
    # ========================================================

    def choose_turn_direction(self):

        if self.left_min >= self.right_min:
            self.turn_dir = 1

        else:
            self.turn_dir = -1

    # ========================================================
    # DIAGNOSTIC
    # ========================================================

    def publish_diag(self, linear, angular):

        payload = {

            "ts": round(time.time(), 3),

            "mode": self.mode,

            "front": round(self.front_min, 3),
            "left": round(self.left_min, 3),
            "right": round(self.right_min, 3),

            "linear": round(linear, 3),
            "angular": round(angular, 3),

            "danger_detected": self.danger_detected,

            "camera_servo_s2":
                CAMERA_VERTICAL_ANGLE,

            "scans": self.scan_count
        }

        msg = String()

        msg.data = json.dumps(payload)

        self.pub_diag.publish(msg)

    # ========================================================
    # LOG
    # ========================================================

    def log_status(self, now):

        if now - self.last_log < LOG_INTERVAL:
            return

        self.last_log = now

        self.get_logger().info(

            f"MODE={self.mode:<18} | "

            f"FRONT={self.front_min:.2f} "

            f"LEFT={self.left_min:.2f} "

            f"RIGHT={self.right_min:.2f} "

            f"DIR={self.turn_dir:+d} "

            f"DANGER={self.danger_detected}"

        )

    # ========================================================
    # STOP
    # ========================================================

    def stop_robot(self):

        self.get_logger().warn(
            "STOP ROBOT"
        )

        for _ in range(15):

            self.send_cmd(0.0, 0.0)

            time.sleep(0.05)


# ============================================================
# MAIN
# ============================================================

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