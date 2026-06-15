import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math

# Plant configuration — DT knows these rates
PLANT_TYPES = {
    'A': {'simulated_growth_rate': 0.0015},
    'B': {'simulated_growth_rate': 0.001125},
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
    (4,  'B', (-2.00, -1.70)),
    (5,  'B', (-1.90, -1.70)),
    # y = -2.70 row
    (6,  'B', (-2.20, -2.70)),
    (7,  'B', (-1.90, -2.70)),
    # Zone A second
    # y = -2.70 row
    (8,  'A', (-0.45, -2.70)),
    (9,  'A', ( 0.25, -2.70)),
    # y = -1.70 row
    (10, 'A', ( 0.25, -1.70)),
    (11, 'A', (-0.45, -1.70)),
    # y = -0.70 row
    (12, 'A', (-0.45, -0.70)),
    (13, 'A', ( 0.1, -0.70)),
    # y = 0.30 row
    (14, 'A', ( 0.25,  0.30)),
    (15, 'A', (-0.45,  0.30)),
]

BASE_POSITION = (0.0, 0.0)
ROBOT_SPEED = 0.22
ROBOT_CAPACITY = 5
EARLY_DEPARTURE_OFFSET = 0.1
GOOD_HARVEST_TOLERANCE = 0.15


class DynamicCropMapNode(Node):
    def __init__(self):
        super().__init__('dynamic_crop_map_node')

        # --- Plant state ---
        self.simulated_ripeness = {p[0]: 0.0 for p in PLANTS}
        self.initialised = {p[0]: False for p in PLANTS}
        self.plant_type = {p[0]: p[1] for p in PLANTS}
        self.plant_pos = {p[0]: p[2] for p in PLANTS}

        # --- Farmer thresholds ---
        self.thresholds = {'A': 0.8, 'B': 0.9}
        self.thresholds_received = False

        # --- Robot state ---
        self.robot_pos = BASE_POSITION
        self.robot_capacity = 0
        self.robot_battery = 100.0

        # --- Phase tracking ---
        self.phase = 'waiting'
        self.initial_scan_complete = False
        self.plants_scanned = set()

        # --- Harvest tracking ---
        self.current_harvest_target = None
        self.awaiting_second_scan = None
        self.waiting_for_harvest_complete = False
        self.last_true_ripeness = {}

        # --- Statistics ---
        self.harvested_total = 0
        self.harvested_good = 0

        # --- Watchdog ---
        self.last_target_time = self.get_clock().now()

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
        self.phase_pub = self.create_publisher(
            String, '/aurafarm/phase', 10
        )
        # --- Subscribers ---
        self.create_subscription(
            String, '/aurafarm/farmer_thresholds',
            self.on_farmer_thresholds, 10
        )
        self.create_subscription(
            String, '/aurafarm/plant_scan',
            self.on_plant_scan, 10
        )
        self.create_subscription(
            String, '/aurafarm/harvest_complete',
            self.on_harvest_complete, 10
        )
        self.create_subscription(
            String, '/aurafarm/robot_status',
            self.on_robot_status, 10
        )
        self.create_subscription(
            String, '/aurafarm/base_arrived',
            self.on_base_arrived, 10
        )
        self.create_subscription(
            String, '/aurafarm/crop_arrival',
            self.on_crop_arrival_harvest, 10
        )

        # --- Timers ---
        self.create_timer(1.0, self.update_simulated_ripeness)
        self.create_timer(1.0, self.publish_crop_map)
        self.create_timer(5.0, self.watchdog_check)

        self.get_logger().info(
            'DynamicCropMapNode started — waiting for farmer thresholds'
        )

    # ================================================================
    # FARMER THRESHOLDS
    # ================================================================
    def on_farmer_thresholds(self, msg: String):
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
            self.send_next_scan_target()

        except Exception as e:
            self.get_logger().error(f'Failed to parse thresholds: {e}')

    # ================================================================
    # INITIAL SCAN
    # ================================================================
    def send_next_scan_target(self):
        for plant_id, _, (x, y) in PLANTS:
            if plant_id not in self.plants_scanned:
                msg = String()
                msg.data = f'{plant_id}:{x}:{y}'
                self.next_target_pub.publish(msg)
                self.last_target_time = self.get_clock().now()
                self.get_logger().info(
                    f'Scan target: plant {plant_id} at ({x}, {y})'
                )
                return

        self.initial_scan_complete = True
        self.phase = 'harvesting'
        self.get_logger().info(
            'Initial scan complete — Phase: HARVESTING'
        )
        phase_msg = String()
        phase_msg.data = 'harvesting'
        self.phase_pub.publish(phase_msg)

        self.send_next_harvest_target()

    def on_plant_scan(self, msg: String):
        try:
            parts = msg.data.split(':')
            plant_id = int(parts[0])
            ripeness = float(parts[1])
        except Exception:
            return

        if self.awaiting_second_scan == plant_id:
            self.process_second_scan(plant_id, ripeness)
            return

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
    # CROP ARRIVAL — only used during harvesting phase
    # ================================================================
    def on_crop_arrival_harvest(self, msg: String):
        if self.phase != 'harvesting':
            return

        try:
            plant_id = int(msg.data)
        except ValueError:
            return

        if plant_id != self.current_harvest_target:
            return

        self.get_logger().info(
            f'Robot arrived at harvest target plant {plant_id} '
            f'— requesting second scan'
        )

        self.awaiting_second_scan = plant_id
        self.waiting_for_harvest_complete = True
        harvest_msg = String()
        harvest_msg.data = f'{plant_id}:HARVEST'
        self.harvest_cmd_pub.publish(harvest_msg)

    # ================================================================
    # SIMULATED RIPENESS GROWTH
    # ================================================================
    def update_simulated_ripeness(self):
        for plant_id, plant_type, _ in PLANTS:
            if not self.initialised[plant_id]:
                continue

            rate = PLANT_TYPES[plant_type]['simulated_growth_rate']
            self.simulated_ripeness[plant_id] += rate

    # ================================================================
    # OPTIMAL PATH CALCULATION
    # ================================================================
    def calculate_optimal_target(self):
        if self.robot_battery < 10.0:
            self.get_logger().warn('Battery below 10% — stopping')
            return None

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

            distance = math.sqrt((px - rx)**2 + (py - ry)**2)
            travel_time = distance / ROBOT_SPEED
            predicted_ripeness = sim_ripe + rate * travel_time

            # Start heading to plant when predicted ripeness reaches
            # threshold - EARLY_DEPARTURE_OFFSET
            trigger = threshold - EARLY_DEPARTURE_OFFSET
            if predicted_ripeness < trigger:
                continue

            score = predicted_ripeness

            if (score > best_score or
                    (score == best_score and distance < best_distance)):
                best_plant = plant_id
                best_score = score
                best_distance = distance

        return best_plant

    def send_next_harvest_target(self):
        self.last_target_time = self.get_clock().now()
        self.waiting_for_harvest_complete = False
        self.awaiting_second_scan = None
        self.current_harvest_target = None

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
        self.current_harvest_target = plant_id

        self.get_logger().info(
            f'Optimal target: plant {plant_id} '
            f'(type {self.plant_type[plant_id]}) '
            f'at ({px}, {py}) — '
            f'simulated ripeness={sim_ripe:.3f}, '
            f'threshold={threshold}'
        )

    # ================================================================
    # SECOND SCAN
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
        self.last_true_ripeness[plant_id] = true_ripeness

        if true_ripeness >= threshold:
            cmd_msg = String()
            cmd_msg.data = f'{plant_id}:CONFIRMED'
            self.harvest_cmd_pub.publish(cmd_msg)
            self.get_logger().info(
                f'Plant {plant_id} confirmed ripe → HARVEST'
            )
        else:
            self.simulated_ripeness[plant_id] = true_ripeness
            cmd_msg = String()
            cmd_msg.data = f'{plant_id}:SKIP'
            self.harvest_cmd_pub.publish(cmd_msg)
            self.get_logger().warn(
                f'Plant {plant_id} not truly ripe '
                f'(true={true_ripeness:.3f} < threshold={threshold}) '
                f'— DT model corrected → SKIP'
            )
            self.send_next_harvest_target()

    # ================================================================
    # HARVEST COMPLETE
    # ================================================================
    def on_harvest_complete(self, msg: String):
        try:
            plant_id = int(msg.data)
        except ValueError:
            return

        self.simulated_ripeness[plant_id] = 0.0
        self.robot_capacity += 1
        self.waiting_for_harvest_complete = False
        self.current_harvest_target = None

        # Statistics
        self.harvested_total += 1
        true_ripe = self.last_true_ripeness.get(plant_id, 0.0)
        threshold = self.thresholds[self.plant_type[plant_id]]
        if abs(true_ripe - threshold) <= GOOD_HARVEST_TOLERANCE:
            self.harvested_good += 1

        self.get_logger().info(
            f'Plant {plant_id} harvested — '
            f'true_ripeness={true_ripe:.3f}, '
            f'capacity: {self.robot_capacity}/{ROBOT_CAPACITY}'
        )
        self._log_stats()

        if self.robot_capacity >= ROBOT_CAPACITY:
            base_msg = String()
            base_msg.data = f'BASE:{BASE_POSITION[0]}:{BASE_POSITION[1]}'
            self.next_target_pub.publish(base_msg)
            self.last_target_time = self.get_clock().now()
            self.get_logger().info(
                'Capacity full — sending robot to base'
            )
            return

        self.send_next_harvest_target()

    # ================================================================
    # BASE ARRIVED
    # ================================================================
    def on_base_arrived(self, msg: String):
        self.robot_capacity = 0
        self.get_logger().info(
            'Robot deposited at base — capacity reset, resuming harvesting'
        )
        self.send_next_harvest_target()

    # ================================================================
    # ROBOT STATUS
    # ================================================================
    def on_robot_status(self, msg: String):
        try:
            parts = msg.data.split(':')
            self.robot_pos = (float(parts[0]), float(parts[1]))
            self.robot_capacity = int(parts[2])
            self.robot_battery = float(parts[3])
        except Exception:
            return

    # ================================================================
    # WATCHDOG
    # ================================================================
    def watchdog_check(self):
        if self.phase != 'harvesting':
            return
        if self.waiting_for_harvest_complete:
            return
        if self.current_harvest_target is not None:
            return
        now = self.get_clock().now()
        elapsed = (now - self.last_target_time).nanoseconds / 1e9
        if elapsed > 15.0:
            self.get_logger().warn(
                f'No target sent in {elapsed:.0f}s — retrying'
            )
            self.send_next_harvest_target()

    # ================================================================
    # STATISTICS
    # ================================================================
    def _log_stats(self):
        if self.harvested_total == 0:
            rate = 0.0
        else:
            rate = self.harvested_good / self.harvested_total * 100.0

        self.get_logger().info(
            f'[STATS] harvested={self.harvested_total}, '
            f'good={self.harvested_good}, '
            f'success_rate={rate:.1f}%'
        )

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
