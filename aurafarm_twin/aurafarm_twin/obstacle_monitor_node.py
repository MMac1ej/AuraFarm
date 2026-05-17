"""Forward-arc LiDAR obstacle detector shared by both sides.

Satisfies project requirement #3 (environmental interaction): a percept
derived from the physical LiDAR is published on a `/twin/*` topic that both
the physical and virtual stacks subscribe to, so the same obstacle is
reflected in both worlds.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker


SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class ObstacleMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('aurafarm_obstacle_monitor')

        self.declare_parameter('obstacle_distance_threshold', 0.35)  # metres
        self.declare_parameter('forward_arc_deg', 60.0)
        self.declare_parameter('frame_id', 'base_scan')

        self._obstacle_pub = self.create_publisher(
            Bool, '/twin/obstacle_detected', 10)
        self._marker_pub = self.create_publisher(
            Marker, '/twin/obstacle_marker', 10)

        self.create_subscription(
            LaserScan, '/physical/scan', self._on_scan, SENSOR_QOS)

        self._last_state: bool | None = None
        self.get_logger().info('Obstacle monitor up: watching /physical/scan.')

    def _on_scan(self, msg: LaserScan) -> None:
        threshold = float(
            self.get_parameter('obstacle_distance_threshold').value)
        arc_deg = float(self.get_parameter('forward_arc_deg').value)
        half_arc = math.radians(arc_deg) / 2.0

        # The TurtleBot3 LiDAR sweeps a full 2π starting at 0 rad (front).
        # We treat angles in [-half_arc, +half_arc] (mod 2π) as "forward".
        min_dist = math.inf
        min_angle = 0.0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # Normalize to (-pi, pi] so the forward window is contiguous.
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if -half_arc <= angle <= half_arc and r < min_dist:
                min_dist = r
                min_angle = angle

        detected = math.isfinite(min_dist) and min_dist < threshold

        # Only emit + log on edge transitions, keeps the log readable.
        if detected != self._last_state:
            self.get_logger().info(
                f'Obstacle {"DETECTED" if detected else "cleared"}'
                + (f' at {min_dist:.2f} m, {math.degrees(min_angle):.0f}°'
                   if detected else ''))
            self._last_state = detected

        self._obstacle_pub.publish(Bool(data=detected))
        self._publish_marker(msg, detected, min_dist, min_angle)

    def _publish_marker(self, scan: LaserScan, detected: bool,
                        dist: float, angle: float) -> None:
        marker = Marker()
        marker.header.stamp = scan.header.stamp
        marker.header.frame_id = (
            scan.header.frame_id
            or str(self.get_parameter('frame_id').value))
        marker.ns = 'aurafarm_obstacle'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD if detected else Marker.DELETE
        if detected:
            marker.pose.position.x = dist * math.cos(angle)
            marker.pose.position.y = dist * math.sin(angle)
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.15
            marker.color.r = 1.0
            marker.color.a = 0.9
        self._marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
