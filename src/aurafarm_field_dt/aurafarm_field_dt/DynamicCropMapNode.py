import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import time

# Plant configuration — DT knows these rates
PLANT_TYPES = {
    'A': {'simulated_growth_rate': 0.05},
    'B': {'simulated_growth_rate': 0.03},
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

BASE_POSITION = (0.0, 0.0)
ROBOT_SPEED = 0.22       # m/s — TurtleBot burger default
ROBOT_CAPACITY = 5       # max fruits before returning to base


class DynamicCropMapNode(Node):
    def __init__(self):
        super().__init__('dynamic_crop_map_node')

        # --- Plant state ---
        self.simulated_ripeness = {p[0]: 0.0 for p in PLANTS}
        self.initialised = {p[0]: False for p in PLANTS}
        self.plant_type = {p[0]: p[1] for p in PLANTS}
        self.plant_pos = {p[0]: p[2] for p in PLANTS}

        # --- Farmer thresholds ---
        self.thresholds = {'A': 0.8, 'B': 0.9}  # defaults
        self.thresholds_received = False

        # --- Robot state ---
        self.robot_pos = BASE_POSITION
        self.robot_capacity = 0
        self.robot_battery = 100.0

        # --- Phase tracking ---
        # 'waiting'  — waiting for farmer thresholds
        # 'scanning' — initial scan in progress
        # 'harvesting' — dynamic harvesting loop
        self.phase = 'waiting'
        self.initial_scan_complete = False
        self.plants_scanned = set()

        # --- Second scan tracking ---
        # When DT sends harvest command, waits for second scan result
        self.awaiting_second_scan = None  # plant_id
        self.second_scan_result = None

        # --- Publishers ---
        self.next_target_pub = self.create_publisher(
            String, '/aurafarm/next_target', 10
        )
        self.harvest_cmd_pub = self.create_publisher(
            String, '/aurafarm/harvest_command', 10
        )
        self.crop_map_pub = self.create_publisher(
            String, '/aurafarm/crop_map', 10
        )

        # --- Subscribers ---
        self.create_subscription(
            String,
            '/aurafarm/farmer_thresholds',
            self.on_farmer_thresholds,
            10
        )
        self.create_subscription(
            String,
            '/aurafarm/plant_scan',
            self.on_plant_scan,
            10
        )
        self.create_subscription(
            String,
            '/aurafarm/harvest_complete',
            self.on_harvest_complete,
            10
        )
        self.create_subscription(
            String,
            '/aurafarm/robot_status',
            self.on_robot_status,
            10
        )

        # --- Timers ---
        # Update simulated ripeness every second
        self.create_timer(1.0, self.update_simulated_ripeness)
        # Publish crop map every second
        self.create_timer(1.0, self.publish_crop_map)

        self.get_logger().info(
            'DynamicCropMapNode started — waiting for farmer thresholds'
        )

    # ================================================================
    # FARMER THRESHOLDS
    # ================================================================
    def on_farmer_thresholds(self, msg: String):
        # Parse "A:0.8,B:0.9"
        try:
            for part in msg.data.split(','):
                plant_type, value = part.split(':')
                self.thresholds[plant_type.strip()] = float(value.strip())

            self.thresholds_received = True
            self.phase = 'scanning'

            self.get_logger().info(
                f'Thresholds received: {self.thresholds}'
            )
            self.get_logger().info(
                'Phase: SCANNING — starting initial scan tour'
            )

            # Send robot to first plant
            self.send_next_scan_target()

        except Exception as e:
            self.get_logger().error(f'Failed to parse thresholds: {e}')

    # ================================================================
    # INITIAL SCAN
    # ================================================================
    def send_next_scan_target(self):
        # Find next unscanned plant
        for plant_id, _, (x, y) in PLANTS:
            if plant_id not in self.plants_scanned:
                msg = String()
                msg.data = f'{plant_id}:{x}:{y}'
                self.next_target_pub.publish(msg)
                self.get_logger().info(
                    f'Scan target: plant {plant_id} at ({x}, {y})'
                )
                return

        # All plants scanned — switch to harvesting phase
        self.initial_scan_complete = True
        self.phase = 'harvesting'
        self.get_logger().info(
            'Initial scan complete — Phase: HARVESTING'
        )
        self.send_next_harvest_target()

    def on_plant_scan(self, msg: String):
        # Parse "plant_id:ripeness"
        try:
            parts = msg.data.split(':')
            plant_id = int(parts[0])
            ripeness = float(parts[1])
        except Exception:
            return

        # Second scan result — DT asked for this before harvesting
        if self.awaiting_second_scan == plant_id:
            self.second_scan_result = ripeness
            self.process_second_scan(plant_id, ripeness)
            return

        # Initial scan result
        if not self.initialised[plant_id]:
            self.simulated_ripeness[plant_id] = ripeness
            self.initialised[plant_id] = True
            self.plants_scanned.add(plant_id)

            self.get_logger().info(
                f'Plant {plant_id} initialised: '
                f'ripeness={ripeness:.3f}, '
                f'type={self.plant_type[plant_id]}'
            )

            if self.phase == 'scanning':
                self.send_next_scan_target()

    # ================================================================
    # SIMULATED RIPENESS GROWTH
    # ================================================================
    def update_simulated_ripeness(self):
        for plant_id, plant_type, _ in PLANTS:
            if self.initialised[plant_id]:
                rate = PLANT_TYPES[plant_type]['simulated_growth_rate']
                self.simulated_ripeness[plant_id] = min(
                    1.0,
                    self.simulated_ripeness[plant_id] + rate
                )

    # ================================================================
    # OPTIMAL PATH CALCULATION
    # ================================================================
    def calculate_optimal_target(self):
        if self.robot_battery < 10.0:
            self.get_logger().warn('Battery below 10% — stopping')
            return None

        # Return to base if at capacity
        if self.robot_capacity >= ROBOT_CAPACITY:
            self.get_logger().info(
                f'Capacity full ({self.robot_capacity}/{ROBOT_CAPACITY})'
                f' — returning to base'
            )
            return 'BASE'

        best_plant = None
        best_score = -1.0
        best_distance = float('inf')

        rx, ry = self.robot_pos

        for plant_id, plant_type, (px, py) in PLANTS:
            if not self.initialised[plant_id]:
                continue

            threshold = self.thresholds.get(plant_type, 0.8)
            sim_ripe = self.simulated_ripeness[plant_id]
            rate = PLANT_TYPES[plant_type]['simulated_growth_rate']

            # Straight line distance
            distance = math.sqrt((px - rx)**2 + (py - ry)**2)
            travel_time = distance / ROBOT_SPEED

            # Predicted ripeness when robot arrives
            predicted_ripeness = min(
                1.0, sim_ripe + rate * travel_time
            )

            # Score: only consider plants at or above threshold
            # (now or by the time robot arrives)
            if predicted_ripeness >= threshold:
                score = predicted_ripeness
            elif sim_ripe >= threshold:
                score = sim_ripe
            else:
                continue  # not worth going yet

            # Pick highest score, distance as tiebreaker
            if (score > best_score or
                    (score == best_score and distance < best_distance)):
                best_plant = plant_id
                best_score = score
                best_distance = distance

        return best_plant

    def send_next_harvest_target(self):
        target = self.calculate_optimal_target()

        if target is None:
            self.get_logger().warn(
                'No target found — all plants below threshold '
                'or battery critical'
            )
            return

        if target == 'BASE':
            msg = String()
            msg.data = f'BASE:{BASE_POSITION[0]}:{BASE_POSITION[1]}'
            self.next_target_pub.publish(msg)
            self.get_logger().info('Sending robot to base to deposit')
            return

        plant_id = target
        _, _, (px, py) = PLANTS[plant_id]
        threshold = self.thresholds[self.plant_type[plant_id]]
        sim_ripe = self.simulated_ripeness[plant_id]

        msg = String()
        msg.data = f'{plant_id}:{px}:{py}'
        self.next_target_pub.publish(msg)

        self.get_logger().info(
            f'Optimal target: plant {plant_id} '
            f'(type {self.plant_type[plant_id]}) '
            f'at ({px}, {py}) — '
            f'simulated ripeness={sim_ripe:.3f}, '
            f'threshold={threshold}'
        )

        # Ask for second scan when robot arrives
        self.awaiting_second_scan = plant_id

        # Also send harvest command so PlantSimulator does second scan
        harvest_msg = String()
        harvest_msg.data = f'{plant_id}:HARVEST'
        self.harvest_cmd_pub.publish(harvest_msg)

    # ================================================================
    # SECOND SCAN — verify true ripeness before harvesting
    # ================================================================
    def process_second_scan(self, plant_id: int, true_ripeness: float):
        plant_type = self.plant_type[plant_id]
        threshold = self.thresholds[plant_type]
        sim_ripe = self.simulated_ripeness[plant_id]

        self.get_logger().info(
            f'Second scan plant {plant_id}: '
            f'true={true_ripeness:.3f}, '
            f'simulated={sim_ripe:.3f}, '
            f'threshold={threshold}'
        )

        self.awaiting_second_scan = None

        if true_ripeness >= threshold:
            # Truly ripe — confirm harvest
            cmd_msg = String()
            cmd_msg.data = f'{plant_id}:CONFIRMED'
            self.harvest_cmd_pub.publish(cmd_msg)
            self.get_logger().info(
                f'Plant {plant_id} confirmed ripe → HARVEST'
            )
        else:
            # DT was wrong — skip this plant, update simulated ripeness
            self.simulated_ripeness[plant_id] = true_ripeness
            cmd_msg = String()
            cmd_msg.data = f'{plant_id}:SKIP'
            self.harvest_cmd_pub.publish(cmd_msg)
            self.get_logger().warn(
                f'Plant {plant_id} not truly ripe '
                f'(true={true_ripeness:.3f} < threshold={threshold}) '
                f'— DT model corrected → SKIP'
            )
            # Recalculate and send next target
            self.send_next_harvest_target()

    # ================================================================
    # HARVEST COMPLETE
    # ================================================================
    def on_harvest_complete(self, msg: String):
        try:
            plant_id = int(msg.data)
        except ValueError:
            return

        # Reset simulated ripeness
        self.simulated_ripeness[plant_id] = 0.0
        self.robot_capacity += 1

        self.get_logger().info(
            f'Plant {plant_id} harvested — '
            f'capacity: {self.robot_capacity}/{ROBOT_CAPACITY}'
        )

        # Send next target
        self.send_next_harvest_target()

    # ================================================================
    # ROBOT STATUS
    # ================================================================
    def on_robot_status(self, msg: String):
        # Parse "x:y:capacity:battery"
        try:
            parts = msg.data.split(':')
            self.robot_pos = (float(parts[0]), float(parts[1]))
            self.robot_capacity = int(parts[2])
            self.robot_battery = float(parts[3])
        except Exception:
            return

    # ================================================================
    # CROP MAP PUBLISHER
    # ================================================================
    def publish_crop_map(self):
        entries = []
        for plant_id, plant_type, (px, py) in PLANTS:
            sim_ripe = self.simulated_ripeness[plant_id]
            threshold = self.thresholds.get(plant_type, 0.8)
            init = 'Y' if self.initialised[plant_id] else 'N'
            entries.append(
                f'{plant_id}:{plant_type}:{sim_ripe:.3f}:'
                f'{threshold}:{init}'
            )

        msg = String()
        msg.data = '|'.join(entries)
        self.crop_map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCropMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()