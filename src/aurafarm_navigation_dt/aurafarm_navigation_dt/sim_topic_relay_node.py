import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry


class SimTopicRelayNode(Node):
    def __init__(self):
        super().__init__('sim_topic_relay')

        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.create_subscription(TFMessage, '/sim/tf', self.on_tf, 100)
        self.create_subscription(Odometry, '/sim/odom', self.on_odom, 10)

        self.get_logger().info('SimTopicRelay started — relaying /sim/tf→/tf and /sim/odom→/odom')

    def on_tf(self, msg: TFMessage):
        self.tf_pub.publish(msg)

    def on_odom(self, msg: Odometry):
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimTopicRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
