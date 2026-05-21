import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import json

LINEAR_FAST   = 0.18
LINEAR_SLOW   = 0.07
ANGULAR_TURN  = 0.50
TURN_DURATION = 1.4

DIST_STOP  = 0.30
DIST_SLOW  = 0.55
DIST_SIDE  = 0.25
LOG_INTERVAL = 0.4


class AutonomousNode(Node):

    def __init__(self):
        super().__init__('autonomous_node')

        self.pub_cmd  = self.create_publisher(Twist,  '/cmd_vel',          10)
        self.pub_diag = self.create_publisher(String, '/robot/diagnostics', 10)

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        # ══ NOUVEAU : écoute /JoyState pour s'arrêter si manuel ══
        self.sub_joy = self.create_subscription(
            Bool, '/JoyState', self.joystate_callback, 10)

        self.active = True   # False = mode manuel, on ne publie plus rien

        self.mode       = "INIT"
        self.turn_until = 0.0
        self.turn_dir   = 1
        self.last_log   = 0.0
        self.scan_count = 0

        self.front_min = 9.99
        self.left_min  = 9.99
        self.right_min = 9.99

        self.get_logger().info("=" * 50)
        self.get_logger().info("  Node autonome démarré")
        self.get_logger().info("=" * 50)

    # ── /JoyState callback ───────────────────────────────────────────────
    def joystate_callback(self, msg: Bool):
        # JoyState=True  → manette active → on arrête l'autonome
        # JoyState=False → mode auto      → on reprend
        self.active = not msg.data
        if not self.active:
            self.send_cmd(0.0, 0.0)   # stop immédiat
            self.get_logger().info("⏸  Autonome suspendu (mode manuel)")
        else:
            self.get_logger().info("▶  Autonome actif")

    # ── Commande moteur ──────────────────────────────────────────────────
    def send_cmd(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x  = float(linear)
        twist.angular.z = float(angular)
        self.pub_cmd.publish(twist)

    # ── Diagnostics ──────────────────────────────────────────────────────
    def publish_diag(self, linear: float, angular: float):
        payload = {
            "ts":      round(time.time(), 3),
            "mode":    self.mode,
            "front":   round(self.front_min, 3),
            "left":    round(self.left_min,  3),
            "right":   round(self.right_min, 3),
            "linear":  round(linear,  3),
            "angular": round(angular, 3),
            "scans":   self.scan_count,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub_diag.publish(msg)

    # ── Parse scan ───────────────────────────────────────────────────────
    @staticmethod
    def _safe_min(values, lo=0.10, hi=3.50):
        filtered = [r for r in values if lo < r < hi and not math.isnan(r)]
        return min(filtered) if filtered else 9.99

    def parse_scan(self, msg: LaserScan):
        ranges = list(msg.ranges)
        n = len(ranges)
        front_idx = list(range(0, 20)) + list(range(n - 20, n))
        left_idx  = list(range(60, 120))
        right_idx = list(range(n - 120, n - 60))
        self.front_min = self._safe_min([ranges[i] for i in front_idx if i < n])
        self.left_min  = self._safe_min([ranges[i] for i in left_idx  if i < n])
        self.right_min = self._safe_min([ranges[i] for i in right_idx if i < n])

    # ── Callback LIDAR ───────────────────────────────────────────────────
    def lidar_callback(self, msg: LaserScan):
        # ══ NOUVEAU : si mode manuel, on ignore le scan ══
        if not self.active:
            return

        now = time.time()
        self.scan_count += 1
        self.parse_scan(msg)

        linear  = 0.0
        angular = 0.0

        if self.mode in ("TOURNE_GAUCHE", "TOURNE_DROITE"):
            if now < self.turn_until:
                angular = ANGULAR_TURN * self.turn_dir
                self.send_cmd(0.0, angular)
                self.publish_diag(0.0, angular)
                self._maybe_log(now)
                return
            else:
                self.mode = "AVANCE"

        if self.front_min < DIST_STOP:
            self.send_cmd(0.0, 0.0)
            self._choose_turn(now)
            self.get_logger().warn(f"⚠  Obstacle à {self.front_min:.2f} m → {self.mode}")
            self.publish_diag(0.0, 0.0)
            self._maybe_log(now)
            return

        if self.front_min < DIST_SLOW:
            self.mode = "AVANCE_LENT"
            linear    = LINEAR_SLOW
            if self.left_min > self.right_min + 0.10:
                angular = 0.15
            elif self.right_min > self.left_min + 0.10:
                angular = -0.15
            self.send_cmd(linear, angular)
            self.publish_diag(linear, angular)
            self._maybe_log(now)
            return

        if self.left_min < DIST_SIDE:
            self.mode = "EVITE_DROITE"
            self.send_cmd(LINEAR_SLOW, -0.30)
            self.publish_diag(LINEAR_SLOW, -0.30)
            self._maybe_log(now)
            return

        if self.right_min < DIST_SIDE:
            self.mode = "EVITE_GAUCHE"
            self.send_cmd(LINEAR_SLOW, 0.30)
            self.publish_diag(LINEAR_SLOW, 0.30)
            self._maybe_log(now)
            return

        self.mode = "AVANCE"
        self.send_cmd(LINEAR_FAST, 0.0)
        self.publish_diag(LINEAR_FAST, 0.0)
        self._maybe_log(now)

    def _choose_turn(self, now: float):
        if self.left_min >= self.right_min:
            self.turn_dir = 1
            self.mode     = "TOURNE_GAUCHE"
        else:
            self.turn_dir = -1
            self.mode     = "TOURNE_DROITE"
        extra = 0.4 if self.front_min < 0.20 else 0.0
        self.turn_until = now + TURN_DURATION + extra

    def _maybe_log(self, now: float):
        if now - self.last_log < LOG_INTERVAL:
            return
        self.last_log = now
        icons = {
            "AVANCE": "▶", "AVANCE_LENT": "▷",
            "TOURNE_GAUCHE": "↰", "TOURNE_DROITE": "↱",
            "EVITE_GAUCHE": "↖", "EVITE_DROITE": "↗",
            "STOP": "■", "INIT": "◉",
        }
        icon = icons.get(self.mode, "?")
        self.get_logger().info(
            f"{icon}  MODE={self.mode:<14} | "
            f"FRONT={self.front_min:5.2f}m  "
            f"LEFT={self.left_min:5.2f}m  "
            f"RIGHT={self.right_min:5.2f}m  "
            f"[scan #{self.scan_count}]"
        )

    def stop_robot(self):
        self.get_logger().warn("■  Arrêt du robot")
        for _ in range(15):
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