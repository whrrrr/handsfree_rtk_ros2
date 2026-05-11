from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from gps_waypoint_nav.waypoint_file import default_waypoint_file


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    enabled = LaunchConfiguration('enabled')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_waypoint_file()),
        DeclareLaunchArgument('enabled', default_value='false'),
        Node(
            package='gps_waypoint_nav',
            executable='waypoint_follower',
            name='gps_waypoint_follower',
            output='screen',
            parameters=[
                params_file,
                {'enabled': enabled},
            ],
        ),
    ])
