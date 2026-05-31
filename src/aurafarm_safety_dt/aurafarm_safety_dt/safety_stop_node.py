import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class SafetyStopNode(Node):
    def __init__(self):
        super().__init__("safety_stop_node")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("input_cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("output_cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("front_angle_deg", 50.0)
        self.declare_parameter("stop_distance", 0.05)
        self.declare_parameter("slow_distance", 0.15)
        self.declare_parameter("enable_slowdown", True)

        self.latest_scan = None
        self.safety_stop_active = False
        self.nearest_front_distance = float("inf")

        scan_topic = self.get_parameter("scan_topic").value
        input_cmd_vel_topic = self.get_parameter("input_cmd_vel_topic").value
        output_cmd_vel_topic = self.get_parameter("output_cmd_vel_topic").value

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            input_cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            output_cmd_vel_topic,
            10
        )

        self.safety_pub = self.create_publisher(
            Bool,
            "/aurafarm/safety_stop",
            10
        )

        self.distance_pub = self.create_publisher(
            Float32,
            "/aurafarm/safety_distance",
            10
        )

        self.get_logger().info("Safety stop node started.")

    def scan_callback(self, msg: LaserScan):
        front_angle_deg = self.get_parameter("front_angle_deg").value
        stop_distance = self.get_parameter("stop_distance").value

        half_angle_rad = math.radians(front_angle_deg / 2.0)

        front_ranges = []

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if -half_angle_rad <= angle <= half_angle_rad:
                if math.isfinite(distance):
                    if msg.range_min <= distance <= msg.range_max:
                        front_ranges.append(distance)

        if len(front_ranges) == 0:
            self.nearest_front_distance = float("inf")
            self.safety_stop_active = False
        else:
            self.nearest_front_distance = min(front_ranges)
            self.safety_stop_active = self.nearest_front_distance <= stop_distance

        safety_msg = Bool()
        safety_msg.data = self.safety_stop_active
        self.safety_pub.publish(safety_msg)

        distance_msg = Float32()
        distance_msg.data = float(self.nearest_front_distance)
        self.distance_pub.publish(distance_msg)

    def cmd_vel_callback(self, msg: Twist):
        stop_distance = self.get_parameter("stop_distance").value
        slow_distance = self.get_parameter("slow_distance").value
        enable_slowdown = self.get_parameter("enable_slowdown").value

        output = Twist()

        # Only block forward motion.
        # This still allows Nav2 to rotate or reverse away from an obstacle.
        moving_forward = msg.linear.x > 0.0

        if self.safety_stop_active and moving_forward:
            output.linear.x = 0.0
            output.linear.y = 0.0
            output.linear.z = 0.0

            output.angular.x = msg.angular.x
            output.angular.y = msg.angular.y
            output.angular.z = msg.angular.z

            self.get_logger().warn(
                f"SAFETY STOP: obstacle at {self.nearest_front_distance:.2f} m"
            )

        elif (
            enable_slowdown
            and moving_forward
            and self.nearest_front_distance < slow_distance
        ):
            # Scale speed between stop_distance and slow_distance.
            distance_range = slow_distance - stop_distance
            scale = (self.nearest_front_distance - stop_distance) / distance_range
            scale = max(0.0, min(1.0, scale))

            output.linear.x = msg.linear.x * scale
            output.linear.y = msg.linear.y
            output.linear.z = msg.linear.z

            output.angular.x = msg.angular.x
            output.angular.y = msg.angular.y
            output.angular.z = msg.angular.z

        else:
            output = msg

        self.cmd_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()