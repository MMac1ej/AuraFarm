import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

# Plant configuration
PLANT_TYPES = {
    'A': {
        'simulated_growth_rate': 0.05,  # ripeness units per second
        'true_growth_offset': 0.2,      # ±20% random offset from simulated
    },
    'B': {
        'simulated_growth_rate': 0.03,
        'true_growth_offset': 0.2,
    }
}

PLANTS = [
    # id, type, (x, y)
    # Zone A
    (0,  'A', (-0.45,  0.30)),
    (1,  'A', (-0.45, -0.70)),
    (2,  'A', (-0.45, -1.70)),
    (3,  'A', (-0.45, -2.70)),
    (4,  'A', ( 0.25,  0.30)),
    (5,  'A', ( 0.25, -0.70)),
    (6,  'A', ( 0.25, -1.70)),
    (7,  'A', ( 0.25, -2.70)),
    # Zone B
    (8,  'B', ( 1.10,  0.30)),
    (9,  'B', ( 1.10, -0.70)),
    (10, 'B', ( 1.10, -1.70)),
    (11, 'B', ( 1.10, -2.70)),
    (12, 'B', ( 1.85,  0.30)),
    (13, 'B', ( 1.85, -0.70)),
    (14, 'B', ( 1.85, -1.70)),
    (15, 'B', ( 1.85, -2.70)),
]


class PlantSimulatorNode(Node):
    def __init__(self):
        super().__init__('plant_simulator_node')

        # true_ripeness[plant_id] = current true ripeness
        self.true_ripeness = {}

        # true_growth_rate[plant_id] = actual growth rate
        # (simulated rate ± random offset)
        self.true_growth_rate = {}

        # Track which plants have been initialised
        # (robot scanned them in initial tour)
        self.initialised = {}

        # Initialise all plants
        for plant_id, plant_type, _ in PLANTS:
            sim_rate = PLANT_TYPES[plant_type]['simulated_growth_rate']
            offset = PLANT_TYPES[plant_type]['true_growth_offset']

            # True growth rate is offset from simulated rate
            true_rate = sim_rate * (
                1.0 + random.uniform(-offset, offset)
            )

            self.true_growth_rate[plant_id] = true_rate
            self.true_ripeness[plant_id] = 0.0
            self.initialised[plant_id] = False

        # Publishes true ripeness when robot scans a plant
        self.scan_pub = self.create_publisher(
            String,
            '/aurafarm/plant_scan',
            10
        )

        # Listens for robot arriving at a plant (initial scan)
        self.arrival_sub = self.create_subscription(
            String,
            '/aurafarm/crop_arrival',
            self.on_crop_arrival,
            10
        )

        # Listens for harvest complete — resets plant to 0.0
        self.harvest_sub = self.create_subscription(
            String,
            '/aurafarm/harvest_complete',
            self.on_harvest_complete,
            10
        )

        # Listens for harvest command — does second scan on arrival
        self.harvest_cmd_sub = self.create_subscription(
            String,
            '/aurafarm/harvest_command',
            self.on_harvest_command,
            10
        )

        # Updates true ripeness every second
        self.timer = self.create_timer(1.0, self.update_ripeness)

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
        # Only grow plants that have been initialised
        for plant_id, _, _ in PLANTS:
            if self.initialised[plant_id]:
                self.true_ripeness[plant_id] = min(
                    1.0,
                    self.true_ripeness[plant_id] +
                    self.true_growth_rate[plant_id]
                )

    def on_crop_arrival(self, msg: String):
        # Robot arrived at plant for initial scan
        # Message format: "plant_id"
        try:
            plant_id = int(msg.data)
        except ValueError:
            return

        if plant_id < 0 or plant_id >= len(PLANTS):
            return

        # Generate initial ripeness randomly
        initial_ripeness = random.uniform(0.0, 0.5)
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
        # DT is asking for a second scan before harvesting
        # Message format: "plant_id:HARVEST"
        parts = msg.data.split(':')
        if len(parts) != 2:
            return

        plant_id = int(parts[0])

        # Publish true ripeness as second scan
        scan_msg = String()
        scan_msg.data = f'{plant_id}:{self.true_ripeness[plant_id]:.3f}'
        self.scan_pub.publish(scan_msg)

        self.get_logger().info(
            f'Second scan plant {plant_id}: '
            f'true ripeness={self.true_ripeness[plant_id]:.3f}'
        )

    def on_harvest_complete(self, msg: String):
        # Robot harvested a plant — reset to 0.0
        try:
            plant_id = int(msg.data)
        except ValueError:
            return

        self.true_ripeness[plant_id] = 0.0
        self.get_logger().info(
            f'Plant {plant_id} harvested — reset to 0.0'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlantSimulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()