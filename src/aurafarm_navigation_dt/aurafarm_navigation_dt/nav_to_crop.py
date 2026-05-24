import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from rclpy.duration import Duration
import time

CROP_POSITIONS = [
    # Zone A
    (-0.45,  0.30),
    (-0.45, -0.70),
    (-0.45, -1.70),
    (-0.45, -2.70),
    ( 0.25,  0.30),
    ( 0.25, -0.70),
    ( 0.25, -1.70),
    ( 0.25, -2.70),
    # Zone B
    ( 1.10,  0.30),
    ( 1.10, -0.70),
    ( 1.10, -1.70),
    ( 1.10, -2.70),
    ( 1.85,  0.30),
    ( 1.85, -0.70),
    ( 1.85, -1.70),
    ( 1.85, -2.70),
]

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

    arrival_pub = node.create_publisher(Int32, '/aurafarm/crop_arrival', 10)
    latest_decision = {'value': None}

    def decision_callback(msg):
        latest_decision['value'] = msg.data
        node.get_logger().info(f'Decision received: {msg.data}')

    node.create_subscription(
        String,
        '/aurafarm/harvest_decision',
        decision_callback,
        10
    )

    nav = BasicNavigator()
    nav.waitUntilNav2Active()

    print('Waiting for sensor and decision nodes...')
    time.sleep(3.0)
    print('Starting crop tour...')

    for crop_id, (x, y) in enumerate(CROP_POSITIONS):
        print(f'Navigating to crop {crop_id + 1} at ({x}, {y})')
        nav.goToPose(make_pose(nav, x, y))

        # Navigate without spin_once interference
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                remaining = Duration.from_msg(
                    feedback.estimated_time_remaining
                ).nanoseconds / 1e9
                print(f'ETA: {remaining:.1f}s')

        result = nav.getResult()

        if result == TaskResult.SUCCEEDED:
            print(f'Arrived at crop {crop_id + 1}!')
            time.sleep(0.5)

            # Publish arrival
            msg = Int32()
            msg.data = crop_id
            arrival_pub.publish(msg)

            # Spin to process decision — only after navigation is complete
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
    rclpy.shutdown()

if __name__ == '__main__':
    main()