# Sim-only relay: bridges Gazebo namespaced topics (/sim/tf, /sim/odom, /sim/scan)
# to the standard Nav2 topics (/tf, /odom, /scan), and forwards Nav2 velocity
# commands (/cmd_vel) back to the Gazebo robot (/sim/cmd_vel).

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class SimTopicRelayNode(Node):
    def __init__(self):
        super().__init__('sim_topic_relay')

        # Sim-only: relay Gazebo TF and odom to standard names for Nav2
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.create_subscription(TFMessage, '/sim/tf', self.on_tf, 100)
        self.create_subscription(Odometry, '/sim/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/sim/scan', self.on_scan, 10)

        # Sim-only: forward Nav2 cmd_vel to the namespaced Gazebo robot
        self.sim_cmd_pub = self.create_publisher(TwistStamped, '/sim/cmd_vel', 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd_vel, 10)

        self.get_logger().info(
            'SimTopicRelay started — '
            '/sim/tf→/tf, /sim/odom→/odom, /sim/scan→/scan, /cmd_vel→/sim/cmd_vel'
        )

    def on_tf(self, msg: TFMessage):
        self.tf_pub.publish(msg)

    def on_odom(self, msg: Odometry):
        self.odom_pub.publish(msg)

    def on_scan(self, msg: LaserScan):
        self.scan_pub.publish(msg)

    def on_cmd_vel(self, msg: TwistStamped):
        self.sim_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimTopicRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
