from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('driver_ids', default_value='1,2'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('wheel_radius_m', default_value='0.04'),
        DeclareLaunchArgument('wheel_base_m', default_value='0.355'),
        DeclareLaunchArgument('max_rpm', default_value='30.0'),
        DeclareLaunchArgument('send_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('cmd_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('watchdog_ms', default_value='500'),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('invert_linear', default_value='false'),
        DeclareLaunchArgument('invert_left', default_value='false'),
        DeclareLaunchArgument('invert_right', default_value='false'),
        DeclareLaunchArgument('swap_left_right', default_value='false'),
        DeclareLaunchArgument('id1_left_sign', default_value='1'),
        DeclareLaunchArgument('id1_right_sign', default_value='1'),
        DeclareLaunchArgument('id2_left_sign', default_value='1'),
        DeclareLaunchArgument('id2_right_sign', default_value='1'),
        Node(
            package='zlac8015d_driver',
            executable='zlac8015d_cmd_vel',
            name='zlac8015d_cmd_vel',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'driver_ids': LaunchConfiguration('driver_ids'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'wheel_radius_m': LaunchConfiguration('wheel_radius_m'),
                'wheel_base_m': LaunchConfiguration('wheel_base_m'),
                'max_rpm': LaunchConfiguration('max_rpm'),
                'send_rate_hz': LaunchConfiguration('send_rate_hz'),
                'cmd_timeout_sec': LaunchConfiguration('cmd_timeout_sec'),
                'watchdog_ms': LaunchConfiguration('watchdog_ms'),
                'enabled': LaunchConfiguration('enabled'),
                'invert_linear': LaunchConfiguration('invert_linear'),
                'invert_left': LaunchConfiguration('invert_left'),
                'invert_right': LaunchConfiguration('invert_right'),
                'swap_left_right': LaunchConfiguration('swap_left_right'),
                'id1_left_sign': LaunchConfiguration('id1_left_sign'),
                'id1_right_sign': LaunchConfiguration('id1_right_sign'),
                'id2_left_sign': LaunchConfiguration('id2_left_sign'),
                'id2_right_sign': LaunchConfiguration('id2_right_sign'),
            }],
        ),
    ])
