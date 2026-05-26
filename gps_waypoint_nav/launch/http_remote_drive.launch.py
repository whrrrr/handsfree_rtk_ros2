from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    esp32_ip = LaunchConfiguration('esp32_ip')
    esp32_port = LaunchConfiguration('esp32_port')
    wheel_base_m = LaunchConfiguration('wheel_base_m')
    max_wheel_speed_mps = LaunchConfiguration('max_wheel_speed_mps')
    min_effective_speed_mps = LaunchConfiguration('min_effective_speed_mps')
    invert_linear = LaunchConfiguration('invert_linear')
    http_host = LaunchConfiguration('http_host')
    http_port = LaunchConfiguration('http_port')

    return LaunchDescription([
        DeclareLaunchArgument('esp32_ip', default_value='192.168.153.239'),
        DeclareLaunchArgument('esp32_port', default_value='8888'),
        DeclareLaunchArgument('wheel_base_m', default_value='0.355'),
        DeclareLaunchArgument('max_wheel_speed_mps', default_value='0.0'),
        DeclareLaunchArgument('min_effective_speed_mps', default_value='0.12'),
        DeclareLaunchArgument('invert_linear', default_value='true'),
        DeclareLaunchArgument('http_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        Node(
            package='diff_drive_udp',
            executable='diff_drive_udp',
            name='diff_drive_udp',
            output='screen',
            parameters=[{
                'esp32_ip': esp32_ip,
                'esp32_port': esp32_port,
                'wheel_base_m': wheel_base_m,
                'max_wheel_speed_mps': max_wheel_speed_mps,
                'min_effective_speed_mps': min_effective_speed_mps,
                'invert_linear': invert_linear,
            }],
        ),
        Node(
            package='gps_waypoint_nav',
            executable='rtk_http_bridge',
            name='rtk_http_bridge',
            output='screen',
            parameters=[{
                'host': http_host,
                'port': http_port,
                'api_path': '/api/command',
                'diff_drive_node': '/diff_drive_udp',
            }],
        ),
    ])
