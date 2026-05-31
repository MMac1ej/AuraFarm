import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarRealNode(Node):
    def __init__(self):
        super().__init__('lidar_real_node')

        # Subscribe to real robot scan
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, 10
        )

        # Republish on namespaced topic
        self.pub = self.create_publisher(
            LaserScan, '/real/scan', 10
        )

        self.get_logger().info(
            'LidarRealNode started — republishing /scan to /real/scan'
        )

    def on_scan(self, msg: LaserScan):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarRealNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()