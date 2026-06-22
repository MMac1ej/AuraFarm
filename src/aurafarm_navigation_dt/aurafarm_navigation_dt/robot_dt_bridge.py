import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity


class RobotDTBridgeNode(Node):
    def __init__(self):
        super().__init__('robot_dt_bridge')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_odom = None

        # Service client to move Gazebo model
        # Needs ros_gz_bridge running for:
        # /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose
        self.set_pose_client = self.create_client(
            SetEntityPose,
            '/world/default/set_pose'
        )

        # Subscribe to real robot odometry
        self.create_subscription(
            Odometry, '/odom', self.on_odom, 10
        )

        # Subscribe to real robot LiDAR
        self.create_subscription(
            LaserScan, '/scan', self.on_scan, 10
        )

        # Publish mirrored odometry to DT topic
        self.dt_odom_pub = self.create_publisher(
            Odometry, '/aurafarm/dt_odom', 10
        )

        # Publish mirrored scan to DT topic
        self.dt_scan_pub = self.create_publisher(
            LaserScan, '/aurafarm/dt_scan', 10
        )

        # Update Gazebo model position at 10Hz
        self.create_timer(0.1, self.update_gz_pose)

        self.get_logger().info(
            'RobotDTBridge started — mirroring real robot to Gazebo at 10Hz'
        )

    def on_odom(self, msg: Odometry):
        self.latest_odom = msg
        self.dt_odom_pub.publish(msg)

        # Broadcast TF for RViz visualisation
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint_real'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def update_gz_pose(self):
        if self.latest_odom is None:
            return

        if not self.set_pose_client.service_is_ready():
            self.get_logger().warn(
                'SetEntityPose service not ready — mirroring paused',
                throttle_duration_sec=5.0
            )
            return

        # Call SetEntityPose service to move the Gazebo model
        request = SetEntityPose.Request()
        request.entity = Entity()
        request.entity.name = 'burger'
        request.entity.type = Entity.MODEL

        request.pose.position.x = self.latest_odom.pose.pose.position.x
        request.pose.position.y = self.latest_odom.pose.pose.position.y
        request.pose.position.z = 0.0
        request.pose.orientation = self.latest_odom.pose.pose.orientation

        # Fire and forget — don't wait for response to avoid blocking
        self.set_pose_client.call_async(request)

    def on_scan(self, msg: LaserScan):
        self.dt_scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDTBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()