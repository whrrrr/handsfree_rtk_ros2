from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('esp32_ip', default_value='192.168.153.239'),
        DeclareLaunchArgument('esp32_port', default_value='8888'),
        DeclareLaunchArgument('wheel_base_m', default_value='0.355'),
        DeclareLaunchArgument('max_wheel_speed_mps', default_value='0.25'),
        DeclareLaunchArgument('min_effective_speed_mps', default_value='0.12'),
        DeclareLaunchArgument('send_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('cmd_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('invert_left', default_value='false'),
        DeclareLaunchArgument('invert_right', default_value='false'),
        DeclareLaunchArgument('swap_wheels', default_value='false'),
        Node(
            package='diff_drive_udp',
            executable='diff_drive_udp',
            name='diff_drive_udp',
            output='screen',
            parameters=[{
                'esp32_ip': LaunchConfiguration('esp32_ip'),
                'esp32_port': LaunchConfiguration('esp32_port'),
                'wheel_base_m': LaunchConfiguration('wheel_base_m'),
                'max_wheel_speed_mps': LaunchConfiguration('max_wheel_speed_mps'),
                'min_effective_speed_mps': LaunchConfiguration('min_effective_speed_mps'),
                'send_rate_hz': LaunchConfiguration('send_rate_hz'),
                'cmd_timeout_sec': LaunchConfiguration('cmd_timeout_sec'),
                'invert_left': LaunchConfiguration('invert_left'),
                'invert_right': LaunchConfiguration('invert_right'),
                'swap_wheels': LaunchConfiguration('swap_wheels'),
            }],
        ),
    ])
