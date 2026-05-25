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
    phase = {'value': 'waiting'}  # waiting / scanning / harvesting

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

    # --- Subscribers ---
    def on_next_target(msg: String):
        # Format: "plant_id:x:y" or "BASE:x:y"
        parts = msg.data.split(':')
        if len(parts) != 3:
            return
        current_target['plant_id'] = parts[0]
        current_target['x'] = float(parts[1])
        current_target['y'] = float(parts[2])
        node.get_logger().info(
            f'New target received: {msg.data}'
        )

    def on_harvest_command(msg: String):
        # Format: "plant_id:CONFIRMED" or "plant_id:SKIP"
        harvest_command['value'] = msg.data
        node.get_logger().info(
            f'Harvest command: {msg.data}'
        )

    def on_farmer_thresholds(msg: String):
        # Thresholds received — start scanning phase
        phase['value'] = 'scanning'
        node.get_logger().info(
            'Thresholds received — starting initial scan'
        )

    node.create_subscription(
        String, '/aurafarm/next_target', on_next_target, 10
    )
    node.create_subscription(
        String, '/aurafarm/harvest_command', on_harvest_command, 10
    )
    node.create_subscription(
        String, '/aurafarm/farmer_thresholds', on_farmer_thresholds, 10
    )

    # --- Nav2 setup ---
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    node.get_logger().info('Nav2 active')

    # Set initial pose
    initial_pose = make_pose(nav, 0.0, 0.0)
    nav.setInitialPose(initial_pose)
    time.sleep(2.0)

    print('Waiting for farmer to set thresholds...')

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

            # Update simulated position while navigating
            robot_pos['x'] = x
            robot_pos['y'] = y
            publish_status()

        return nav.getResult()

    # --- Helper: wait for message with timeout ---
    def wait_for(state_dict, key, timeout=10.0):
        state_dict[key] = None
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if state_dict[key] is not None:
                return True
        return False

    # ================================================================
    # MAIN LOOP
    # ================================================================

    # Step 1 — wait for farmer thresholds
    print('Waiting for farmer thresholds...')
    while phase['value'] == 'waiting':
        rclpy.spin_once(node, timeout_sec=0.1)

    print('Thresholds received — waiting for first scan target...')
    time.sleep(1.0)

    # Step 2 — initial scan + harvesting loop
    while True:
        # Check battery
        if battery['level'] < 10.0:
            node.get_logger().warn(
                'Battery below 10% — stopping'
            )
            print('Battery critical — stopping harvesting tour')
            break

        # Wait for next target from DT
        current_target['plant_id'] = None
        print('Waiting for next target from DT...')
        deadline = time.time() + 30.0
        while current_target['plant_id'] is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() > deadline:
                print('No target received in 30s — waiting...')
                deadline = time.time() + 30.0

        plant_id = current_target['plant_id']
        tx = current_target['x']
        ty = current_target['y']

        # Navigate to target
        result = navigate_to(tx, ty, f'plant {plant_id}')

        if result != TaskResult.SUCCEEDED:
            node.get_logger().warn(
                f'Failed to reach plant {plant_id} — skipping'
            )
            # Tell DT we skipped so it recalculates
            skip_msg = String()
            skip_msg.data = f'{plant_id}:SKIP'
            harvest_complete_pub.publish(skip_msg)
            continue

        # Arrived at target
        robot_pos['x'] = tx
        robot_pos['y'] = ty
        publish_status()

        # Handle BASE return
        if plant_id == 'BASE':
            print('Arrived at base — depositing fruits')
            capacity['count'] = 0
            time.sleep(1.0)
            publish_status()
            node.get_logger().info('Deposited — resuming harvesting')
            continue

        plant_id_int = int(plant_id)
        print(f'Arrived at plant {plant_id_int}!')
        time.sleep(0.5)

        # Publish arrival for initial scan
        arrival_msg = String()
        arrival_msg.data = str(plant_id_int)
        arrival_pub.publish(arrival_msg)

        # Wait for harvest command from DT
        # (DT will send CONFIRMED or SKIP after second scan)
        print(f'Waiting for harvest decision on plant {plant_id_int}...')
        received = wait_for(harvest_command, 'value', timeout=10.0)

        if not received:
            print(
                f'No harvest command for plant {plant_id_int} '
                f'— moving on'
            )
            continue

        cmd_parts = harvest_command['value'].split(':')
        cmd_plant_id = int(cmd_parts[0])
        cmd_action = cmd_parts[1]

        if cmd_plant_id != plant_id_int:
            # Command is for a different plant — ignore
            continue

        if cmd_action == 'CONFIRMED':
            print(f'Plant {plant_id_int} — HARVESTING')
            capacity['count'] += 1

            # Publish harvest complete so DT resets plant
            complete_msg = String()
            complete_msg.data = str(plant_id_int)
            harvest_complete_pub.publish(complete_msg)

            node.get_logger().info(
                f'Harvested plant {plant_id_int} — '
                f'capacity: {capacity["count"]}/{ROBOT_CAPACITY}'
            )

            # Simulate battery drain per harvest
            battery['level'] -= 2.0
            publish_status()

        elif cmd_action == 'SKIP':
            print(
                f'Plant {plant_id_int} — SKIP '
                f'(not truly ripe yet)'
            )

    print('Harvesting tour ended')
    rclpy.shutdown()


if __name__ == '__main__':
    main()