import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, String
from rclpy.duration import Duration


# 7 crop positions chosen from the map
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

    # Regular node for publishing crop arrival events
    node = Node('nav_to_crop_node')

    # Publisher: tells other nodes which crop the robot just arrived at
    # Other nodes (sensor, DT) will subscribe to this
    arrival_pub = node.create_publisher(Int32, '/aurafarm/crop_arrival', 10)

    # Subscriber: listens for harvest decision from DT before moving to next crop
    latest_decision = {'value': None}

    def decision_callback(msg):
        latest_decision['value'] = msg.data
        node.get_logger().info(f'Received decision: {msg.data}')

    node.create_subscription(
        String,
        '/aurafarm/harvest_decision',
        decision_callback,
        10
    )

    nav = BasicNavigator()

    # Set initial pose — robot starts at origin
    initial_pose = make_pose(nav, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # Wait for Nav2 to be ready
    nav.waitUntilNav2Active()
    node.get_logger().info('Nav2 is active, starting crop tour...')

    # Visit each crop position in order
    for crop_id, (x, y) in enumerate(CROP_POSITIONS):
        node.get_logger().info(f'Navigating to crop {crop_id + 1} at ({x}, {y})')

        goal = make_pose(nav, x, y)
        nav.goToPose(goal)

        # Monitor navigation progress
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                remaining = Duration.from_msg(
                    feedback.estimated_time_remaining
                ).nanoseconds / 1e9
                node.get_logger().info(
                    f'Crop {crop_id + 1}: ETA {remaining:.1f}s'
                )
            # Spin once to process any incoming messages
            rclpy.spin_once(node, timeout_sec=0.1)

        result = nav.getResult()

        if result == TaskResult.SUCCEEDED:
            node.get_logger().info(
                f'Arrived at crop {crop_id + 1}!'
            )

            # Publish arrival event so sensor node can generate a reading
            arrival_msg = Int32()
            arrival_msg.data = crop_id
            arrival_pub.publish(arrival_msg)

            # Wait for decision from DT (5 seconds)
            latest_decision['value'] = None
            wait_count = 0
            while latest_decision['value'] is None and wait_count < 50:
                rclpy.spin_once(node, timeout_sec=0.1)
                wait_count += 1

            if latest_decision['value'] is not None:
                node.get_logger().info(
                    f'Crop {crop_id + 1} decision: {latest_decision["value"]}'
                )
            else:
                node.get_logger().warn(
                    f'No decision received for crop {crop_id + 1}, moving on'
                )

        elif result == TaskResult.FAILED:
            node.get_logger().error(
                f'Failed to reach crop {crop_id + 1}, skipping'
            )

        elif result == TaskResult.CANCELED:
            node.get_logger().warn(f'Navigation to crop {crop_id + 1} canceled')

    node.get_logger().info('Crop tour complete!')
    nav.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()