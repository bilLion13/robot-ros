import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Int32, String, Bool
from geometry_msgs.msg import Twist
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
import json
import time


class TelemetryNode(Node):

    def __init__(self):
        super().__init__('telemetry_node')

        # ── InfluxDB ─────────────────────────────────────────────────────
        self.client = InfluxDBClient(
            url="https://us-east-1-1.aws.cloud2.influxdata.com",
            token="X69mccEPDdx-0j3GGWPjSgYaKcJHLs3NHTWAbO4Qpp970uvsfwQJLmYk-6ZHyuDGreekVBuwueXYnW3I6jzXyw==",
            org="Wb"
        )
        self.write_api = self.client.write_api(write_options=SYNCHRONOUS)
        self.bucket    = "robot-ros-pi"

        # ── État interne ─────────────────────────────────────────────────
        self.battery     = 0
        self.hole        = 0
        self.linear      = 0.0
        self.angular     = 0.0
        self.robot_mode  = "manual"
        self.front_min   = 9.99
        self.left_min    = 9.99
        self.right_min   = 9.99
        self.lidar_mode  = "INIT"

        # ── Subscriptions ────────────────────────────────────────────────
        self.create_subscription(UInt16, '/battery',          self.cb_battery,  10)
        self.create_subscription(Int32,  '/hole_detected',    self.cb_hole,     10)
        self.create_subscription(Twist,  '/cmd_vel',          self.cb_cmdvel,   10)
        self.create_subscription(String, '/robot/diagnostics',self.cb_diag,     10)
        self.create_subscription(Bool,   '/JoyState',         self.cb_joystate, 10)

        # ── Timer — envoie tout sur une ligne toutes les 2 secondes ─────
        self.timer = self.create_timer(2.0, self.send_all)

        self.get_logger().info("TELEMETRY NODE READY → robot_data")

    # ── Callbacks ────────────────────────────────────────────────────────

    def cb_battery(self, msg):
        self.battery = int(msg.data)

    def cb_hole(self, msg):
        self.hole = int(msg.data)
        # Envoi immédiat si trou détecté — pas d'attente du timer
        if self.hole == 1:
            self.send_all()
            self.get_logger().info("🕳  Trou détecté — envoi immédiat")

    def cb_cmdvel(self, msg):
        self.linear  = round(msg.linear.x,  3)
        self.angular = round(msg.angular.z, 3)

    def cb_diag(self, msg):
        try:
            d = json.loads(msg.data)
            self.front_min  = d.get("front",  9.99)
            self.left_min   = d.get("left",   9.99)
            self.right_min  = d.get("right",  9.99)
            self.lidar_mode = d.get("mode",   "INIT")
        except Exception:
            pass

    def cb_joystate(self, msg):
        # JoyState=True → manette | False → autonome
        self.robot_mode = "manual" if msg.data else "auto"

    # ── Envoi unique toutes les 2 s ──────────────────────────────────────

    def send_all(self):
        try:
            point = (
                Point("robot_data")

                # Identification
                .tag("mode", self.robot_mode)
                .tag("lidar_mode", self.lidar_mode)

                # Batterie
                .field("battery",      self.battery)

                # Détection
                .field("hole_detected", self.hole)

                # Déplacement
                .field("linear",       self.linear)
                .field("angular",      self.angular)

                # Lidar distances
                .field("front_dist",   self.front_min)
                .field("left_dist",    self.left_min)
                .field("right_dist",   self.right_min)
            )

            self.write_api.write(bucket=self.bucket, record=point)

            self.get_logger().info(
                f"robot_data | mode={self.robot_mode} "
                f"bat={self.battery} hole={self.hole} "
                f"lin={self.linear} ang={self.angular} "
                f"F={self.front_min:.2f} L={self.left_min:.2f} R={self.right_min:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"Erreur InfluxDB : {e}")


def main():
    rclpy.init()
    node = TelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()