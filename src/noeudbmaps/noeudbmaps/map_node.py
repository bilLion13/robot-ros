import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry


class MapNode(Node):

    def __init__(self):

        super().__init__('map_node')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info("MAP NODE READY")

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.get_logger().info(
            f"Robot position -> X:{x:.2f} Y:{y:.2f}"
        )


def main():

    rclpy.init()

    node = MapNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()