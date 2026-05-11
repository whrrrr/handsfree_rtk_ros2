from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    params_file = LaunchConfiguration('params_file')
    enabled = LaunchConfiguration('enabled')

    default_params = PathJoinSubstitution([
        FindPackageShare('gps_waypoint_nav'),
        'config',
        'waypoints.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/HFRobotRTK'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('enabled', default_value='false'),
        Node(
            package='handsfree_rtk',
            executable='ros_driver_um98x',
            name='handsfree_rtk',
            output='screen',
            parameters=[{
                'port': port,
                'baudrate': baudrate,
                'frame_id': 'gps',
            }],
        ),
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
