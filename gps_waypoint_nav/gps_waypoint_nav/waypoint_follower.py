import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String
from visualization_msgs.msg import Marker

from gps_waypoint_nav.geo import enu_distance_and_bearing, latlon_to_enu, wrap_angle


@dataclass
class Waypoint:
    latitude: float
    longitude: float
    name: str


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')

        self.declare_parameter('enabled', False)
        self.declare_parameter('gnss_topic', 'handsfree/rtk/gnss')
        self.declare_parameter('cog_topic', 'handsfree/rtk/cog')
        self.declare_parameter('heading_topic', 'handsfree/rtk/heading')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('waypoint_latitudes', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('waypoint_longitudes', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('waypoint_names', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('arrival_radius_m', 0.5)
        self.declare_parameter('max_linear_speed', 5.0)
        self.declare_parameter('min_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 4.0)
        self.declare_parameter('heading_kp', 10.0)
        self.declare_parameter('slow_radius_m', 2.0)
        self.declare_parameter('large_heading_error_rad', 1.2)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('require_fix', True)
        self.declare_parameter('frame_id', 'gps_map')

        self.enabled = bool(self.get_parameter('enabled').value)
        self.arrival_radius_m = float(self.get_parameter('arrival_radius_m').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.slow_radius_m = float(self.get_parameter('slow_radius_m').value)
        self.large_heading_error_rad = float(self.get_parameter('large_heading_error_rad').value)
        self.require_fix = bool(self.get_parameter('require_fix').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.waypoints = self._load_waypoints()
        self.active_index = 0
        self.origin = None
        self.latest_fix = None
        self.latest_xy = None
        self.previous_xy = None
        self.latest_heading = None
        self.cog_heading = None
        self.true_heading = None
        self.track_path = Path()
        self.track_path.header.frame_id = self.frame_id

        gnss_topic = self.get_parameter('gnss_topic').value
        cog_topic = self.get_parameter('cog_topic').value
        heading_topic = self.get_parameter('heading_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.create_subscription(NavSatFix, gnss_topic, self._on_fix, 20)
        self.create_subscription(Float64, cog_topic, self._on_cog, 20)
        self.create_subscription(Float64, heading_topic, self._on_heading, 20)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.target_pub = self.create_publisher(NavSatFix, '~/target_fix', 10)
        self.waypoint_path_pub = self.create_publisher(Path, '~/waypoint_path', 1)
        self.track_path_pub = self.create_publisher(Path, '~/track_path', 1)
        self.marker_pub = self.create_publisher(Marker, '~/target_marker', 1)

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.timer = self.create_timer(1.0 / max(1.0, control_rate_hz), self._control_loop)

        self.get_logger().info(
            'Loaded %d waypoints, enabled=%s, gnss_topic=%s, cmd_vel_topic=%s'
            % (len(self.waypoints), self.enabled, gnss_topic, cmd_vel_topic))

    def _on_set_parameters(self, params):
        waypoint_latitudes = None
        waypoint_longitudes = None
        waypoint_names = None
        for param in params:
            if param.name == 'enabled':
                enabled = bool(param.value)
                if enabled and not self.enabled:
                    if waypoint_latitudes is not None or waypoint_longitudes is not None or waypoint_names is not None:
                        self.waypoints = self._waypoints_from_values(
                            waypoint_latitudes, waypoint_longitudes, waypoint_names)
                    else:
                        self.waypoints = self._load_waypoints()
                    self.active_index = 0
                    self.get_logger().info(
                        'Navigation enabled, reloaded %d waypoints' % len(self.waypoints))
                elif not enabled and self.enabled:
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info('Navigation disabled, stop published once')
                self.enabled = enabled
            elif param.name == 'arrival_radius_m':
                self.arrival_radius_m = float(param.value)
            elif param.name == 'max_linear_speed':
                self.max_linear_speed = float(param.value)
            elif param.name == 'min_linear_speed':
                self.min_linear_speed = float(param.value)
            elif param.name == 'max_angular_speed':
                self.max_angular_speed = float(param.value)
            elif param.name == 'heading_kp':
                self.heading_kp = float(param.value)
            elif param.name == 'slow_radius_m':
                self.slow_radius_m = float(param.value)
            elif param.name == 'large_heading_error_rad':
                self.large_heading_error_rad = float(param.value)
            elif param.name == 'require_fix':
                self.require_fix = bool(param.value)
            elif param.name == 'waypoint_latitudes':
                waypoint_latitudes = list(param.value)
            elif param.name == 'waypoint_longitudes':
                waypoint_longitudes = list(param.value)
            elif param.name == 'waypoint_names':
                waypoint_names = list(param.value)
        if waypoint_latitudes is not None or waypoint_longitudes is not None or waypoint_names is not None:
            self.waypoints = self._waypoints_from_values(
                waypoint_latitudes, waypoint_longitudes, waypoint_names)
        return SetParametersResult(successful=True)

    def _waypoints_from_values(self, latitudes=None, longitudes=None, names=None):
        if latitudes is None:
            latitudes = list(self.get_parameter_or(
                'waypoint_latitudes',
                Parameter('waypoint_latitudes', Parameter.Type.DOUBLE_ARRAY, [])
            ).value)
        if longitudes is None:
            longitudes = list(self.get_parameter_or(
                'waypoint_longitudes',
                Parameter('waypoint_longitudes', Parameter.Type.DOUBLE_ARRAY, [])
            ).value)
        if names is None:
            names = list(self.get_parameter_or(
                'waypoint_names',
                Parameter('waypoint_names', Parameter.Type.STRING_ARRAY, [])
            ).value)

        if len(latitudes) != len(longitudes):
            raise ValueError('waypoint_latitudes and waypoint_longitudes must have the same length')
        waypoints = []
        for i, (lat, lon) in enumerate(zip(latitudes, longitudes)):
            name = names[i] if i < len(names) else 'wp_%d' % (i + 1)
            waypoints.append(Waypoint(float(lat), float(lon), str(name)))
        return waypoints

    def _load_waypoints(self):
        latitudes = list(self.get_parameter_or(
            'waypoint_latitudes',
            Parameter('waypoint_latitudes', Parameter.Type.DOUBLE_ARRAY, [])
        ).value)
        longitudes = list(self.get_parameter_or(
            'waypoint_longitudes',
            Parameter('waypoint_longitudes', Parameter.Type.DOUBLE_ARRAY, [])
        ).value)
        names = list(self.get_parameter_or(
            'waypoint_names',
            Parameter('waypoint_names', Parameter.Type.STRING_ARRAY, [])
        ).value)
        if len(latitudes) != len(longitudes):
            raise ValueError('waypoint_latitudes and waypoint_longitudes must have the same length')
        waypoints = []
        for i, (lat, lon) in enumerate(zip(latitudes, longitudes)):
            name = names[i] if i < len(names) else 'wp_%d' % (i + 1)
            waypoints.append(Waypoint(float(lat), float(lon), str(name)))
        return waypoints

    def _on_fix(self, msg):
        if self.require_fix and msg.status.status == NavSatStatus.STATUS_NO_FIX:
            self.latest_fix = msg
            return

        if self.origin is None:
            self.origin = (msg.latitude, msg.longitude)
            self._publish_waypoint_path()
            self.get_logger().info(
                'Set local ENU origin: lat=%.8f lon=%.8f' % self.origin)

        self.latest_fix = msg
        self.previous_xy = self.latest_xy
        self.latest_xy = latlon_to_enu(
            msg.latitude, msg.longitude, self.origin[0], self.origin[1])
        self._update_heading_from_motion()
        self._append_track_pose(msg)

    def _on_cog(self, msg):
        self.cog_heading = math.radians(float(msg.data))

    def _on_heading(self, msg):
        self.true_heading = math.radians(float(msg.data))

    def _update_heading_from_motion(self):
        if self.previous_xy is None or self.latest_xy is None:
            return
        dx = self.latest_xy[0] - self.previous_xy[0]
        dy = self.latest_xy[1] - self.previous_xy[1]
        if math.hypot(dx, dy) > 0.05:
            self.latest_heading = math.atan2(dx, dy)

    def _append_track_pose(self, fix_msg):
        if self.latest_xy is None:
            return
        pose = PoseStamped()
        pose.header.stamp = fix_msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.latest_xy[0]
        pose.pose.position.y = self.latest_xy[1]
        pose.pose.orientation.w = 1.0
        self.track_path.header.stamp = self.get_clock().now().to_msg()
        self.track_path.poses.append(pose)
        if len(self.track_path.poses) > 2000:
            self.track_path.poses = self.track_path.poses[-2000:]

    def _control_loop(self):
        if not self.enabled:
            self._publish_status('disabled')
            return
        if not self.waypoints:
            self._publish_stop('no waypoints configured')
            return
        if self.latest_fix is None or self.latest_xy is None:
            self._publish_stop('waiting for GNSS fix')
            return
        if self.require_fix and self.latest_fix.status.status == NavSatStatus.STATUS_NO_FIX:
            self._publish_stop('GNSS no fix')
            return
        if self.active_index >= len(self.waypoints):
            self._publish_stop('mission complete')
            return

        target = self.waypoints[self.active_index]
        target_xy = latlon_to_enu(
            target.latitude, target.longitude, self.origin[0], self.origin[1])
        distance, target_bearing = enu_distance_and_bearing(self.latest_xy, target_xy)

        self._publish_target_fix(target)
        self._publish_target_marker(target_xy, target.name)
        self.track_path_pub.publish(self.track_path)

        if distance <= self.arrival_radius_m:
            self.get_logger().info(
                'Arrived waypoint %d/%d (%s), distance=%.2fm'
                % (self.active_index + 1, len(self.waypoints), target.name, distance))
            self.active_index += 1
            self._publish_stop('arrived %s' % target.name)
            return

        heading, heading_source = self._select_heading()
        if heading is None:
            self._publish_stop('waiting for heading / COG / motion heading')
            return

        heading_error = wrap_angle(target_bearing - heading)
        linear = self._linear_speed(distance, abs(heading_error))
        angular = max(-self.max_angular_speed,
                      min(self.max_angular_speed, -self.heading_kp * heading_error))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)
        self._publish_status(
            'tracking %s %d/%d: distance=%.2fm heading_error=%.1fdeg v=%.2f w=%.2f source=%s'
            % (target.name, self.active_index + 1, len(self.waypoints), distance,
               math.degrees(heading_error), linear, angular, heading_source))

    def _select_heading(self):
        if self.true_heading is not None:
            return self.true_heading, 'heading'
        if self.cog_heading is not None:
            return self.cog_heading, 'cog'
        if self.latest_heading is not None:
            return self.latest_heading, 'motion'
        return None, 'none'

    def _linear_speed(self, distance, abs_heading_error):
        if abs_heading_error > self.large_heading_error_rad:
            return 0.0
        speed = self.max_linear_speed
        if distance < self.slow_radius_m:
            speed = self.max_linear_speed * max(0.0, distance / self.slow_radius_m)
            speed = max(self.min_linear_speed, speed)
        heading_scale = max(0.25, 1.0 - abs_heading_error / self.large_heading_error_rad)
        return min(self.max_linear_speed, speed * heading_scale)

    def _publish_stop(self, reason):
        self.cmd_pub.publish(Twist())
        self._publish_status(reason)

    def _publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def _publish_target_fix(self, waypoint):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = waypoint.latitude
        msg.longitude = waypoint.longitude
        self.target_pub.publish(msg)

    def _publish_waypoint_path(self):
        if self.origin is None:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.frame_id
        for waypoint in self.waypoints:
            x, y = latlon_to_enu(
                waypoint.latitude, waypoint.longitude, self.origin[0], self.origin[1])
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.waypoint_path_pub.publish(path)

    def _publish_target_marker(self, target_xy, name):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = 'gps_waypoints'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target_xy[0]
        marker.pose.position.y = target_xy[1]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.2
        marker.color.a = 0.9
        marker.text = name
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
