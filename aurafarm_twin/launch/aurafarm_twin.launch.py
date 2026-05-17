"""Bring up the aurafarm_twin nodes (and optionally the TurtleBot3 Gazebo sim).

Example:
    ros2 launch aurafarm_twin aurafarm_twin.launch.py
    ros2 launch aurafarm_twin aurafarm_twin.launch.py start_gazebo:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_gazebo = LaunchConfiguration('start_gazebo')
    enable_bridge = LaunchConfiguration('enable_bridge')
    sync_rate_hz = LaunchConfiguration('sync_rate_hz')
    low_battery_threshold = LaunchConfiguration('low_battery_threshold')
    obstacle_distance_threshold = LaunchConfiguration(
        'obstacle_distance_threshold')
    forward_arc_deg = LaunchConfiguration('forward_arc_deg')

    declared_args = [
        DeclareLaunchArgument(
            'start_gazebo', default_value='true',
            description='Also launch turtlebot3_gazebo empty world.'),
        DeclareLaunchArgument(
            'enable_bridge', default_value='true',
            description='If false, mediator drops all messages.'),
        DeclareLaunchArgument(
            'sync_rate_hz', default_value='5.0',
            description='State sync republish rate (Hz).'),
        DeclareLaunchArgument(
            'low_battery_threshold', default_value='0.20',
            description='Fraction (0-1) below which a low-battery warning '
                        'is logged.'),
        DeclareLaunchArgument(
            'obstacle_distance_threshold', default_value='0.35',
            description='Forward-arc distance (m) under which an obstacle '
                        'is flagged.'),
        DeclareLaunchArgument(
            'forward_arc_deg', default_value='60.0',
            description='Width of the forward LiDAR window (degrees).'),
    ]

    # TurtleBot3 model is selected via env var — Burger for this project.
    set_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Optional Gazebo bringup. Wrapped in try/except so that if
    # turtlebot3_gazebo isn't installed (e.g. running against the real
    # robot only) we still come up cleanly.
    try:
        tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_share, 'launch',
                             'empty_world.launch.py')),
            condition=IfCondition(start_gazebo),
        )
        gazebo_group = GroupAction([gazebo_launch])
    except Exception:
        gazebo_group = GroupAction([])  # no-op

    mediator = Node(
        package='aurafarm_twin',
        executable='mediator',
        name='aurafarm_mediator',
        output='screen',
        parameters=[{'enable_bridge': enable_bridge}],
    )

    state_sync = Node(
        package='aurafarm_twin',
        executable='state_sync',
        name='aurafarm_state_sync',
        output='screen',
        parameters=[{
            'sync_rate_hz': sync_rate_hz,
            'low_battery_threshold': low_battery_threshold,
        }],
    )

    obstacle_monitor = Node(
        package='aurafarm_twin',
        executable='obstacle_monitor',
        name='aurafarm_obstacle_monitor',
        output='screen',
        parameters=[{
            'obstacle_distance_threshold': obstacle_distance_threshold,
            'forward_arc_deg': forward_arc_deg,
        }],
    )

    return LaunchDescription([
        *declared_args,
        set_model,
        gazebo_group,
        mediator,
        state_sync,
        obstacle_monitor,
    ])
