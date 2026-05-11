import rclpy

from gps_waypoint_nav.waypoint_file import clear_waypoints, default_waypoint_file


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('gps_waypoint_clear')
    try:
        node.declare_parameter('output_file', default_waypoint_file())
        output_file = node.get_parameter('output_file').value
        count = clear_waypoints(output_file)
        node.get_logger().info('Cleared %d waypoints from %s' % (count, output_file))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
