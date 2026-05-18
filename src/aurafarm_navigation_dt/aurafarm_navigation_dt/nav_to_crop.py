import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
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
    nav = BasicNavigator()

    initial_pose = make_pose(nav, 0.0, 0.0)
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()

    for crop_id, (x, y) in enumerate(CROP_POSITIONS):
        print(f'Navigating to crop {crop_id + 1} at ({x}, {y})')
        nav.goToPose(make_pose(nav, x, y))

        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            if feedback:
                remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f'ETA: {remaining:.1f}s')

        result = nav.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f'Arrived at crop {crop_id + 1}!')
        elif result == TaskResult.FAILED:
            print(f'Failed to reach crop {crop_id + 1}, skipping')
        elif result == TaskResult.CANCELED:
            print(f'Navigation to crop {crop_id + 1} canceled')

    print('Crop tour complete!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()