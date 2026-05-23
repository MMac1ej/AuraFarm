import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32, Int32, String
from rclpy.duration import Duration
import time


# The safety stop node should sit between Nav2 and the robot:
#   Nav2 controller_server -> /cmd_vel_nav -> safety_stop_node -> /cmd_vel -> robot
#
# This file does not publish Twist commands directly. It sends goals to Nav2.
# Therefore, the velocity-topic link to the safety node is done in your Nav2
# configuration or launch file by making Nav2 publish to /cmd_vel_nav instead
# of /cmd_vel.

CROP_POSITIONS = [
    (-1.5,  0.0),
    ( 0.4,  0.8),
    ( 0.9, -1.0),
    (-2.0, -3.0),
    (-2.4, -0.5),
    (-1.3, -1.5),
    (-1.0, -3.0),
]

SAFETY_STOP_TOPIC = '/aurafarm/safety_stop'
SAFETY_DISTANCE_TOPIC = '/aurafarm/safety_distance'


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

    node.declare_parameter('cancel_if_safety_stop_exceeds_sec', 0.0)
    node.declare_parameter('safety_log_period_sec', 1.0)

    arrival_pub = node.create_publisher(Int32, '/aurafarm/crop_arrival', 10)

    latest_decision = {'value': None}
    safety_state = {
        'stop_active': False,
        'distance': float('inf'),
        'stop_started_at': None,
        'last_log_at': 0.0,
    }

    def decision_callback(msg):
        latest_decision['value'] = msg.data
        node.get_logger().info(f'Decision received: {msg.data}')

    def safety_stop_callback(msg):
        now = time.time()

        if msg.data and not safety_state['stop_active']:
            safety_state['stop_started_at'] = now
            node.get_logger().warn('Safety stop became active.')

        if not msg.data and safety_state['stop_active']:
            safety_state['stop_started_at'] = None
            node.get_logger().info('Safety stop cleared.')

        safety_state['stop_active'] = msg.data

    def safety_distance_callback(msg):
        safety_state['distance'] = msg.data

    node.create_subscription(
        String,
        '/aurafarm/harvest_decision',
        decision_callback,
        10
    )

    node.create_subscription(
        Bool,
        SAFETY_STOP_TOPIC,
        safety_stop_callback,
        10
    )

    node.create_subscription(
        Float32,
        SAFETY_DISTANCE_TOPIC,
        safety_distance_callback,
        10
    )

    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    node.get_logger().info('Nav2 is active.')
    node.get_logger().info(
        'Required velocity wiring: Nav2 /cmd_vel -> /cmd_vel_nav, '
        'then safety_stop_node /cmd_vel_nav -> /cmd_vel.'
    )

    print('Waiting for sensor, safety, and decision nodes...')
    time.sleep(3.0)
    print('Starting crop tour...')

    for crop_id, (x, y) in enumerate(CROP_POSITIONS):
        print(f'Navigating to crop {crop_id + 1} at ({x}, {y})')
        nav.goToPose(make_pose(nav, x, y))

        while not nav.isTaskComplete():
            
            rclpy.spin_once(node, timeout_sec=0.05)

            feedback = nav.getFeedback()
            if feedback:
                remaining = Duration.from_msg(
                    feedback.estimated_time_remaining
                ).nanoseconds / 1e9

                now = time.time()
                log_period = node.get_parameter(
                    'safety_log_period_sec'
                ).value

                if now - safety_state['last_log_at'] >= log_period:
                    safety_state['last_log_at'] = now
                    if safety_state['stop_active']:
                        print(
                            f'ETA: {remaining:.1f}s | SAFETY STOP active | '
                            f'obstacle: {safety_state["distance"]:.2f} m'
                        )
                    else:
                        print(
                            f'ETA: {remaining:.1f}s | safety clear | '
                            f'nearest front obstacle: '
                            f'{safety_state["distance"]:.2f} m'
                        )

            cancel_after = node.get_parameter(
                'cancel_if_safety_stop_exceeds_sec'
            ).value

            if (
                cancel_after > 0.0
                and safety_state['stop_active']
                and safety_state['stop_started_at'] is not None
                and time.time() - safety_state['stop_started_at'] > cancel_after
            ):
                node.get_logger().warn(
                    'Safety stop has been active too long. Cancelling current goal.'
                )
                nav.cancelTask()
                break

        result = nav.getResult()

        if result == TaskResult.SUCCEEDED:
            print(f'Arrived at crop {crop_id + 1}!')
            time.sleep(0.5)

            msg = Int32()
            msg.data = crop_id
            arrival_pub.publish(msg)

            latest_decision['value'] = None
            deadline = time.time() + 10.0
            while time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if latest_decision['value'] is not None:
                    break

            if latest_decision['value'] is None:
                print(f'No decision received for crop {crop_id + 1}, moving on')

        elif result == TaskResult.FAILED:
            print(f'Failed to reach crop {crop_id + 1}, skipping')
        elif result == TaskResult.CANCELED:
            print(f'Navigation to crop {crop_id + 1} canceled')

    print('Crop tour complete!')
    nav.lifecycleShutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
