import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

from gps_waypoint_nav.waypoint_file import append_waypoint, default_waypoint_file


class CaptureWaypoint(Node):
    def __init__(self):
        super().__init__('gps_waypoint_capture')

        self.declare_parameter('gnss_topic', 'handsfree/rtk/gnss')
        self.declare_parameter('output_file', default_waypoint_file())
        self.declare_parameter('name', '')
        self.declare_parameter('require_fix', True)
        self.declare_parameter('timeout_sec', 10.0)

        self.gnss_topic = self.get_parameter('gnss_topic').value
        self.output_file = self.get_parameter('output_file').value
        self.name = self.get_parameter('name').value
        self.require_fix = bool(self.get_parameter('require_fix').value)
        timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.done = False
        self.success = False
        self.deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)

        self.create_subscription(NavSatFix, self.gnss_topic, self._on_fix, 10)
        self.create_timer(0.2, self._on_timer)
        self.get_logger().info(
            'Waiting for one GNSS fix on %s, output_file=%s' % (self.gnss_topic, self.output_file))

    def _on_fix(self, msg):
        if self.done:
            return
        if self.require_fix and msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return

        name = self.name.strip() or None
        saved_name, count = append_waypoint(self.output_file, msg.latitude, msg.longitude, name)
        self.get_logger().info(
            'Saved waypoint %d (%s): lat=%.8f lon=%.8f'
            % (count, saved_name, msg.latitude, msg.longitude))
        self.done = True
        self.success = True

    def _on_timer(self):
        if self.done:
            return
        if self.get_clock().now().nanoseconds > self.deadline:
            self.get_logger().error('Timed out waiting for GNSS fix.')
            self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = CaptureWaypoint()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        success = node.success
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1
