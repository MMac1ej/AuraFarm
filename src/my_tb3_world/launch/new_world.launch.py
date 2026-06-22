#!/usr/bin/env python3
# Launches the AuraFarm Gazebo world (new_world.world) with the TurtleBot3 Burger
# spawned under the 'sim' namespace — so its scan and odom appear on /sim/scan and
# /sim/odom, separate from the real robot's /scan. Also starts the ros_gz_bridge
# for the SetEntityPose service used by robot_dt_bridge to mirror the real robot's
# position into Gazebo.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    my_pkg_share     = get_package_share_directory('my_tb3_world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(my_pkg_share, 'worlds', 'new_world.world')

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r -s -v2 {world}', 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Wrap spawn_turtlebot3 in 'sim' namespace so turtlebot3_gazebo's own bridge
    # publishes to /sim/scan instead of /scan — no conflict with the real robot's /scan.
    spawn_turtlebot_cmd = GroupAction([
        PushRosNamespace('sim'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='set_pose_bridge',
            arguments=[
                '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose'
            ],
            output='screen',
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
