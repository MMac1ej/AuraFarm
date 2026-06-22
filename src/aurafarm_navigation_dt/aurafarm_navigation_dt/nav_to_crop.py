# Navigation executor: drives the robot to scan and harvest targets issued by
# DynamicCropMapNode via Nav2. Publishes crop_arrival on reach, harvest_complete
# on confirmed harvest, and robot_status (position/capacity/battery) after each move.

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.duration import Duration
import time
import math

BASE_POSITION = (0.0, 0.0)
ROBOT_SPEED = 0.22
ROBOT_CAPACITY = 5
ARRIVAL_RADIUS = 0.2


def make_pose(nav, x, y):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    node = Node('nav_to_crop_node')

    # --- State ---
    current_target = {'plant_id': None, 'x': None, 'y': None}
    harvest_command = {'value': None}
    robot_pos = {'x': 0.0, 'y': 0.0}
    capacity = {'count': 0}
    battery = {'level': 100.0}
    phase = {'value': 'waiting'}

    # --- Publishers ---
    arrival_pub = node.create_publisher(
        String, '/aurafarm/crop_arrival', 10
    )
    harvest_complete_pub = node.create_publisher(
        String, '/aurafarm/harvest_complete', 10
    )
    robot_status_pub = node.create_publisher(
        String, '/aurafarm/robot_status', 10
    )
    base_arrived_pub = node.create_publisher(
        String, '/aurafarm/base_arrived', 10
    )

    # --- Subscribers ---
    def on_next_target(msg: String):
        parts = msg.data.split(':')
        if len(parts) != 3:
            return
        current_target['plant_id'] = parts[0]
        current_target['x'] = float(parts[1])
        current_target['y'] = float(parts[2])
        node.get_logger().info(f'New target received: {msg.data}')

    def on_harvest_command(msg: String):
        # Only store CONFIRMED or SKIP — HARVEST is for PlantSimulatorNode
        if ':HARVEST' in msg.data and ':CONFIRMED' not in msg.data:
            return
        harvest_command['value'] = msg.data
        node.get_logger().info(f'Harvest command: {msg.data}')

    def on_farmer_thresholds(msg: String):
        phase['value'] = 'scanning'
        node.get_logger().info(
            'Thresholds received — starting initial scan'
        )

    def on_phase_change(msg: String):
        phase['value'] = msg.data
        node.get_logger().info(f'Phase changed to: {msg.data}')

    node.create_subscription(
        String, '/aurafarm/next_target', on_next_target, 10
    )
    node.create_subscription(
        String, '/aurafarm/harvest_command', on_harvest_command, 10
    )
    node.create_subscription(
        String, '/aurafarm/farmer_thresholds', on_farmer_thresholds, 10
    )
    node.create_subscription(
        String, '/aurafarm/phase', on_phase_change, 10
    )

    # --- Nav2 setup ---
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    node.get_logger().info('Nav2 active')

    # --- Helper: publish robot status ---
    def publish_status():
        msg = String()
        msg.data = (
            f'{robot_pos["x"]:.2f}:'
            f'{robot_pos["y"]:.2f}:'
            f'{capacity["count"]}:'
            f'{battery["level"]:.1f}'
        )
        robot_status_pub.publish(msg)

    # --- Helper: navigate to position ---
    def navigate_to(x, y, label):
        node.get_logger().info(f'Navigating to {label} at ({x}, {y})')
        nav.goToPose(make_pose(nav, x, y))

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                remaining = Duration.from_msg(
                    feedback.estimated_time_remaining
                ).nanoseconds / 1e9
                print(f'ETA to {label}: {remaining:.1f}s')

                # Proximity check
                current_x = feedback.current_pose.pose.position.x
                current_y = feedback.current_pose.pose.position.y
                distance = math.sqrt(
                    (current_x - x)**2 + (current_y - y)**2
                )
                if distance <= ARRIVAL_RADIUS:
                    nav.cancelTask()
                    robot_pos['x'] = current_x
                    robot_pos['y'] = current_y
                    publish_status()
                    node.get_logger().info(
                        f'Within {ARRIVAL_RADIUS}m of {label} '
                        f'— arrived (distance={distance:.2f}m)'
                    )
                    return TaskResult.SUCCEEDED

        robot_pos['x'] = x
        robot_pos['y'] = y
        publish_status()
        return nav.getResult()

    # --- Helper: wait for next target ---
    def wait_for_target(timeout=60.0):
        # Don't clear if target already received while processing previous plant
        if current_target['plant_id'] is not None:
            return current_target.copy()

        deadline = time.time() + timeout
        while current_target['plant_id'] is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() > deadline:
                print('No target received — waiting...')
                deadline = time.time() + timeout
        return current_target.copy()

    # --- Helper: process harvest logic ---
    def process_harvest(plant_id_int):
        print(
            f'Waiting for harvest decision on '
            f'plant {plant_id_int}...'
        )
        harvest_command['value'] = None
        deadline = time.time() + 15.0
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if harvest_command['value'] is not None:
                break

        if harvest_command['value'] is None:
            print(f'No harvest command — moving on')
            return

        cmd_parts = harvest_command['value'].split(':')
        cmd_plant_id = int(cmd_parts[0])
        cmd_action = cmd_parts[1]

        if cmd_plant_id != plant_id_int:
            print(f'Command mismatch — moving on')
            return

        if cmd_action == 'CONFIRMED':
            print(f'Plant {plant_id_int} — HARVESTING')
            capacity['count'] += 1

            complete_msg = String()
            complete_msg.data = str(plant_id_int)
            harvest_complete_pub.publish(complete_msg)

            battery['level'] -= 2.0
            publish_status()

            node.get_logger().info(
                f'Harvested plant {plant_id_int} — '
                f'capacity: {capacity["count"]}/{ROBOT_CAPACITY}'
            )

        elif cmd_action == 'SKIP':
            print(f'Plant {plant_id_int} — SKIP')

    # ================================================================
    # MAIN LOOP
    # ================================================================

    # Step 1 — wait for farmer thresholds
    print('Waiting for farmer thresholds...')
    while phase['value'] == 'waiting':
        rclpy.spin_once(node, timeout_sec=0.1)

    print('Starting tour — waiting for first target from DT...')
    time.sleep(1.0)

    # Step 2 — main navigation loop
    while True:

        # Check battery
        if battery['level'] < 10.0:
            print('Battery critical — stopping')
            break

        # Wait for next target from DT
        target = wait_for_target()
        target = wait_for_target()
        current_target['plant_id'] = None  # clear after reading
        plant_id = target['plant_id']
        tx = target['x']
        ty = target['y']
        
        # Navigate to target
        result = navigate_to(tx, ty, f'plant {plant_id}')

        if result != TaskResult.SUCCEEDED:
            node.get_logger().warn(
                f'Failed to reach {plant_id} — skipping'
            )
            continue

        # Arrived
        print(f'Arrived at {plant_id}!')
        time.sleep(0.5)

        # Handle BASE return
        if plant_id == 'BASE':
            print('Depositing fruits at base')
            capacity['count'] = 0
            publish_status()
            time.sleep(1.0)

            # Notify DT deposit complete
            base_msg = String()
            base_msg.data = 'done'
            base_arrived_pub.publish(base_msg)

            node.get_logger().info('Deposited — notified DT')
            continue

        plant_id_int = int(plant_id)

        # Publish arrival — triggers scan in PlantSimulator
        arrival_msg = String()
        arrival_msg.data = str(plant_id_int)
        arrival_pub.publish(arrival_msg)

        # SCANNING phase — spin briefly to catch phase change
        if phase['value'] == 'scanning':
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.1)
                if phase['value'] == 'harvesting':
                    break

            if phase['value'] == 'scanning':
                # Still scanning — move to next scan target
                print(
                    f'Plant {plant_id_int} scanned — '
                    f'waiting for next scan target...'
                )
                continue
            else:
                # Phase just switched — fall through to harvest logic
                print(
                    f'Phase switched to harvesting at '
                    f'plant {plant_id_int} — processing harvest...'
                )

        # HARVESTING phase
        process_harvest(plant_id_int)

    print('Harvesting tour ended')
    rclpy.shutdown()


if __name__ == '__main__':
    main()