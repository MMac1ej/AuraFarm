"""Bidirectional bridge between the physical TurtleBot3 and its Gazebo twin.

Satisfies project requirement #1 (bidirectional communication):
  physical -> twin :  /physical/scan   -> /sim/scan
                      /physical/odom   -> /sim/odom
  twin -> physical :  /sim/cmd_vel     -> /physical/cmd_vel
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


# TurtleBot3 publishes sensor topics with BEST_EFFORT reliability. If we
# subscribe with the default RELIABLE QoS, DDS treats the endpoints as
# incompatible and we silently receive zero messages. Match the publisher.
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

# cmd_vel is a control topic — we want every command delivered.
CONTROL_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class MediatorNode(Node):
    def __init__(self) -> None:
        super().__init__('aurafarm_mediator')

        # Lets an operator pause the bridge at runtime via
        #   ros2 param set /aurafarm_mediator enable_bridge false
        self.declare_parameter('enable_bridge', True)

        # --- Publishers (twin side) ---
        self._sim_scan_pub = self.create_publisher(
            LaserScan, '/sim/scan', SENSOR_QOS)
        self._sim_odom_pub = self.create_publisher(
            Odometry, '/sim/odom', SENSOR_QOS)

        # --- Publisher (physical side) ---
        self._physical_cmd_pub = self.create_publisher(
            Twist, '/physical/cmd_vel', CONTROL_QOS)

        # --- Subscribers ---
        self.create_subscription(
            LaserScan, '/physical/scan', self._on_physical_scan, SENSOR_QOS)
        self.create_subscription(
            Odometry, '/physical/odom', self._on_physical_odom, SENSOR_QOS)
        self.create_subscription(
            Twist, '/sim/cmd_vel', self._on_sim_cmd_vel, CONTROL_QOS)

        self.get_logger().info(
            'Mediator up: bridging /physical <-> /sim '
            '(scan, odom -> twin; cmd_vel -> physical).')

    # ----- callbacks -----------------------------------------------------

    def _bridge_enabled(self) -> bool:
        return bool(self.get_parameter('enable_bridge').value)

    def _on_physical_scan(self, msg: LaserScan) -> None:
        if self._bridge_enabled():
            self._sim_scan_pub.publish(msg)

    def _on_physical_odom(self, msg: Odometry) -> None:
        if self._bridge_enabled():
            self._sim_odom_pub.publish(msg)

    def _on_sim_cmd_vel(self, msg: Twist) -> None:
        if self._bridge_enabled():
            self._physical_cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MediatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
