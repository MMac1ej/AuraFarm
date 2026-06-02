import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

# Plant configuration
PLANT_TYPES = {
    'A': {
        'simulated_growth_rate': 0.002,
        'true_growth_offset': 0.2,
    },
    'B': {
        'simulated_growth_rate': 0.0015,
        'true_growth_offset': 0.2,
    }
}

PLANTS = [
    # Zone B first (robot starts at ~(-2, 0), Zone B is closest)
    # y = 0.30 row
    (0,  'B', (-2.20,  0.30)),
    (1,  'B', (-1.90,  0.30)),
    # y = -0.70 row
    (2,  'B', (-2.20, -0.70)),
    (3,  'B', (-1.90, -0.70)),
    # y = -1.70 row
    (4,  'B', (-2.20, -1.70)),
    (5,  'B', (-1.90, -1.70)),
    # y = -2.70 row
    (6,  'B', (-2.20, -2.70)),
    (7,  'B', (-1.90, -2.70)),
    # Zone A second
    # y = 0.30 row
    (8,  'A', ( 0.25,  0.30)),
    (9,  'A', (-0.45,  0.30)),
    # y = -0.70 row
    (10, 'A', ( 0.25, -0.70)),
    (11, 'A', (-0.45, -0.70)),
    # y = -1.70 row
    (12, 'A', ( 0.25, -1.70)),
    (13, 'A', (-0.45, -1.70)),
    # y = -2.70 row
    (14, 'A', ( 0.25, -2.70)),
    (15, 'A', (-0.45, -2.70)),
]


class PlantSimulatorNode(Node):
    def __init__(self):
        super().__init__('plant_simulator_node')

        self.true_ripeness = {}
        self.true_growth_rate = {}
        self.initialised = {}

        for plant_id, plant_type, _ in PLANTS:
            sim_rate = PLANT_TYPES[plant_type]['simulated_growth_rate']
            offset = PLANT_TYPES[plant_type]['true_growth_offset']
            true_rate = sim_rate * (
                1.0 + random.uniform(-offset, offset)
            )
            self.true_growth_rate[plant_id] = true_rate
            self.true_ripeness[plant_id] = 0.0
            self.initialised[plant_id] = False

        self.scan_pub = self.create_publisher(
            String, '/aurafarm/plant_scan', 10
        )
        self.create_subscription(
            String, '/aurafarm/crop_arrival',
            self.on_crop_arrival, 10
        )
        self.create_subscription(
            String, '/aurafarm/harvest_complete',
            self.on_harvest_complete, 10
        )
        self.create_subscription(
            String, '/aurafarm/harvest_command',
            self.on_harvest_command, 10
        )

        self.true_map_pub = self.create_publisher(
            String, '/aurafarm/true_ripeness_map', 10
        )
        self.create_timer(1.0, self.publish_true_map)

        self.create_timer(1.0, self.update_ripeness)

        self.get_logger().info('PlantSimulatorNode started')
        self.get_logger().info(
            'True growth rates (offset from simulated):'
        )
        for plant_id, plant_type, _ in PLANTS:
            sim = PLANT_TYPES[plant_type]['simulated_growth_rate']
            true = self.true_growth_rate[plant_id]
            self.get_logger().info(
                f'  Plant {plant_id} (Type {plant_type}): '
                f'simulated={sim:.4f}, true={true:.4f}'
            )

    def update_ripeness(self):
        # Only grow plants that have been scanned
        for plant_id, _, _ in PLANTS:
            if self.initialised[plant_id]:
                self.true_ripeness[plant_id] = min(
                    1.0,
                    self.true_ripeness[plant_id] +
                    self.true_growth_rate[plant_id]
                )

    def on_crop_arrival(self, msg: String):
        try:
            plant_id = int(msg.data)
        except ValueError:
            return

        # Guard — do not reset if already initialised
        if self.initialised[plant_id]:
            return

        # Generate initial ripeness from scan
        initial_ripeness = random.uniform(0.0, 0.1)
        self.true_ripeness[plant_id] = initial_ripeness
        self.initialised[plant_id] = True

        # Publish scan result to DT
        scan_msg = String()
        scan_msg.data = f'{plant_id}:{initial_ripeness:.3f}'
        self.scan_pub.publish(scan_msg)

        self.get_logger().info(
            f'Initial scan plant {plant_id}: '
            f'ripeness={initial_ripeness:.3f}'
        )

    def on_harvest_command(self, msg: String):
        # Second scan — publish true ripeness
        parts = msg.data.split(':')
        if len(parts) != 2:
            return

        try:
            plant_id = int(parts[0])
        except ValueError:
            return

        scan_msg = String()
        scan_msg.data = f'{plant_id}:{self.true_ripeness[plant_id]:.3f}'
        self.scan_pub.publish(scan_msg)

        self.get_logger().info(
            f'Second scan plant {plant_id}: '
            f'true ripeness={self.true_ripeness[plant_id]:.3f}'
        )

    def on_harvest_complete(self, msg: String):
        try:
            plant_id = int(msg.data)
        except ValueError:
            return

        # Reset plant — starts growing again from 0.0
        self.true_ripeness[plant_id] = 0.0
        # Keep initialised=True so it grows again
        # DT will also reset its simulated value on harvest_complete

        self.get_logger().info(
            f'Plant {plant_id} harvested — reset to 0.0'
        )

    def publish_true_map(self):
        entries = []
        for plant_id, plant_type, _ in PLANTS:
            if self.initialised[plant_id]:
                entries.append(
                    f'{plant_id}:{plant_type}:{self.true_ripeness[plant_id]:.3f}'
                )
        if entries:
            msg = String()
            msg.data = '|'.join(entries)
            self.true_map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlantSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()