import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class RobotDTBridgeNode(Node):
    def __init__(self):
        super().__init__('robot_dt_bridge')

        # TF broadcaster — moves the Gazebo model to match real robot
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to real robot odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.on_odom,
            10
        )

        # Subscribe to real robot LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.on_scan,
            10
        )

        # Publish mirrored odometry to DT topic
        self.dt_odom_pub = self.create_publisher(
            Odometry,
            '/aurafarm/dt_odom',
            10
        )

        # Publish mirrored scan to DT topic
        self.dt_scan_pub = self.create_publisher(
            LaserScan,
            '/aurafarm/dt_scan',
            10
        )

        self.get_logger().info('RobotDTBridge started — mirroring real robot to Gazebo')

    def on_odom(self, msg: Odometry):
        # Mirror odometry to DT topic
        self.dt_odom_pub.publish(msg)

        # Broadcast TF transform so Gazebo model moves to match real robot
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'

        # Copy position from real robot odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        # Copy orientation from real robot odometry
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f'DT position updated: '
            f'({msg.pose.pose.position.x:.2f}, '
            f'{msg.pose.pose.position.y:.2f})'
        )

    def on_scan(self, msg: LaserScan):
        # Mirror LiDAR scan to DT topic
        self.dt_scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDTBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()