from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from gps_waypoint_nav.waypoint_file import default_waypoint_file


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    arrival_radius_m = LaunchConfiguration('arrival_radius_m')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_waypoint_file()),
        DeclareLaunchArgument('arrival_radius_m', default_value='0.5'),
        Node(
            package='gps_waypoint_nav',
            executable='waypoint_follower',
            name='gps_waypoint_follower',
            output='screen',
            parameters=[
                params_file,
                {
                    'enabled': True,
                    'max_linear_speed': 0.0,
                    'min_linear_speed': 0.0,
                    'max_angular_speed': 0.0,
                    'arrival_radius_m': arrival_radius_m,
                },
            ],
        ),
    ])
