# Republishes the Gazebo simulation scan from /sim/scan to /aurafarm/sim_scan
# for separate monitoring/visualization of the virtual robot's LiDAR data.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarSimNode(Node):
    def __init__(self):
        super().__init__('lidar_sim_node')

        # Subscribe to Gazebo simulation scan
        # Gazebo bridges its scan to /scan but we remap it
        self.create_subscription(
            LaserScan, '/sim/scan', self.on_scan, 10
        )

        # Republish on namespaced topic
        self.pub = self.create_publisher(
            LaserScan, '/aurafarm/sim_scan', 10
        )

        self.get_logger().info(
            'LidarSimNode started — republishing /sim/scan to /aurafarm/sim_scan'
        )

    def on_scan(self, msg: LaserScan):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()