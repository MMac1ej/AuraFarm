import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math


class LidarMergerNode(Node):
    def __init__(self):
        super().__init__('lidar_merger_node')

        self.real_scan = None
        self.sim_scan = None

        # Subscribe to both scans
        self.create_subscription(
            LaserScan, '/real/scan', self.on_real_scan, 10
        )
        self.create_subscription(
            LaserScan, '/aurafarm/sim_scan', self.on_sim_scan, 10
        )

        # Publish merged scan — Nav2 will use this
        self.merged_pub = self.create_publisher(
            LaserScan, '/aurafarm/merged_scan', 10
        )

        # Merge and publish at 10Hz
        self.create_timer(0.1, self.merge_and_publish)

        self.get_logger().info(
            'LidarMergerNode started — merging real + sim scans'
        )

    def on_real_scan(self, msg: LaserScan):
        self.real_scan = msg

    def on_sim_scan(self, msg: LaserScan):
        self.sim_scan = msg

    def merge_and_publish(self):
        # Need at least real scan to publish
        if self.real_scan is None:
            return

        # If no sim scan yet, just republish real scan
        if self.sim_scan is None:
            self.merged_pub.publish(self.real_scan)
            return

        # Build merged scan based on real scan structure
        merged = LaserScan()
        merged.header = self.real_scan.header
        merged.angle_min = self.real_scan.angle_min
        merged.angle_max = self.real_scan.angle_max
        merged.angle_increment = self.real_scan.angle_increment
        merged.time_increment = self.real_scan.time_increment
        merged.scan_time = self.real_scan.scan_time
        merged.range_min = self.real_scan.range_min
        merged.range_max = self.real_scan.range_max

        real_ranges = list(self.real_scan.ranges)
        sim_ranges = list(self.sim_scan.ranges)
        merged_ranges = []

        num_beams = len(real_ranges)

        for i in range(num_beams):
            real_r = real_ranges[i]
            real_valid = (
                math.isfinite(real_r) and
                self.real_scan.range_min <= real_r <= self.real_scan.range_max
            )

            # Find corresponding angle in sim scan
            angle = self.real_scan.angle_min + i * self.real_scan.angle_increment
            sim_idx = int(
                (angle - self.sim_scan.angle_min) /
                self.sim_scan.angle_increment
            )

            sim_valid = False
            sim_r = float('inf')
            if 0 <= sim_idx < len(sim_ranges):
                sim_r = sim_ranges[sim_idx]
                sim_valid = (
                    math.isfinite(sim_r) and
                    self.sim_scan.range_min <= sim_r <= self.sim_scan.range_max
                )

            # Take minimum distance — closest obstacle wins
            if real_valid and sim_valid:
                merged_ranges.append(min(real_r, sim_r))
            elif real_valid:
                merged_ranges.append(real_r)
            elif sim_valid:
                merged_ranges.append(sim_r)
            else:
                merged_ranges.append(float('inf'))

        merged.ranges = merged_ranges
        self.merged_pub.publish(merged)

        self.get_logger().debug(
            f'Merged scan published — {num_beams} beams'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LidarMergerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()