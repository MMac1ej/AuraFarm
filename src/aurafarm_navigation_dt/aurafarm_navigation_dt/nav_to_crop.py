import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from rclpy.duration import Duration

CROP_POSITIONS = [
    (-1.5, -1.0),
    ( 0.5,  1.0),
    ( 0.9, -1.0),
    (-2.0, -3.0),
    (-2.4, -0.5),
    (-1.0, -1.2),
    (-1.0, -3.0),
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

    # Publishes crop ID when robot arrives
    arrival_pub = node.create_publisher(Int32, '/aurafarm/crop_arrival', 10)

    # Listens for harvest decision before moving to next crop
    latest_decision = {'value': None}

    def decision_callback(msg):
        latest_decision['value'] = msg.data
        node.get_logger().info(f'Decision received: {msg.data}')

    node.create_subscription(String, '/aurafarm/harvest_decision', decision_callback, 10)

    nav = BasicNavigator()
    nav.setInitialPose(make_pose(nav, 0.0, 0.0))
    nav.waitUntilNav2Active()

    for crop_id, (x, y) in enumerate(CROP_POSITIONS):
        print(f'Navigating to crop {crop_id + 1} at ({x}, {y})')
        nav.goToPose(make_pose(nav, x, y))

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f'ETA: {remaining:.1f}s')
            rclpy.spin_once(node, timeout_sec=0.1)

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Arrived at crop {crop_id + 1}!')

            # Publish arrival so sensor node generates a reading
            msg = Int32()
            msg.data = crop_id
            arrival_pub.publish(msg)

            # Wait up to 5 seconds for decision
            latest_decision['value'] = None
            for _ in range(50):
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