import math
import socket

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class DiffDriveUdp(Node):
    def __init__(self):
        super().__init__('diff_drive_udp')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('esp32_ip', '192.168.153.239')
        self.declare_parameter('esp32_port', 8888)
        self.declare_parameter('wheel_base_m', 0.355)
        self.declare_parameter('max_wheel_speed_mps', 0.0)
        self.declare_parameter('min_effective_speed_mps', 0.12)
        self.declare_parameter('send_rate_hz', 10.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('swap_wheels', False)
        self.declare_parameter('enabled', True)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.esp32_ip = self.get_parameter('esp32_ip').value
        self.esp32_port = int(self.get_parameter('esp32_port').value)
        self.wheel_base_m = float(self.get_parameter('wheel_base_m').value)
        self.max_wheel_speed_mps = abs(float(self.get_parameter('max_wheel_speed_mps').value))
        self.min_effective_speed_mps = abs(float(self.get_parameter('min_effective_speed_mps').value))
        self.send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.swap_wheels = bool(self.get_parameter('swap_wheels').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.add_on_set_parameters_callback(self.on_set_parameters)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_cmd_time = None
        self.target_left = 0.0
        self.target_right = 0.0
        self.last_sent = None

        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        period = 1.0 / max(self.send_rate_hz, 0.1)
        self.create_timer(period, self.on_timer)

        self.get_logger().info(
            'UDP diff drive ready: %s -> %s:%d, wheel_base=%.3fm, max=%s, min=%.2fm/s'
            % (self.cmd_vel_topic, self.esp32_ip, self.esp32_port, self.wheel_base_m,
               self._max_speed_text(), self.min_effective_speed_mps))

    def _max_speed_text(self):
        if self.max_wheel_speed_mps <= 0.0:
            return 'unlimited'
        return '%.2fm/s' % self.max_wheel_speed_mps

    def on_set_parameters(self, params):
        for param in params:
            if param.name == 'esp32_ip':
                ip = str(param.value).strip()
                try:
                    socket.inet_aton(ip)
                except OSError:
                    return SetParametersResult(successful=False, reason='invalid esp32_ip')
                self.esp32_ip = ip
            elif param.name == 'esp32_port':
                port = int(param.value)
                if port <= 0 or port > 65535:
                    return SetParametersResult(successful=False, reason='invalid esp32_port')
                self.esp32_port = port
            elif param.name == 'enabled':
                self.enabled = bool(param.value)
            elif param.name == 'wheel_base_m':
                self.wheel_base_m = float(param.value)
            elif param.name == 'max_wheel_speed_mps':
                self.max_wheel_speed_mps = abs(float(param.value))
            elif param.name == 'min_effective_speed_mps':
                self.min_effective_speed_mps = abs(float(param.value))
            elif param.name == 'cmd_timeout_sec':
                self.cmd_timeout_sec = float(param.value)
            elif param.name == 'invert_left':
                self.invert_left = bool(param.value)
            elif param.name == 'invert_right':
                self.invert_right = bool(param.value)
            elif param.name == 'swap_wheels':
                self.swap_wheels = bool(param.value)

        self.get_logger().info(
            'updated target: %s:%d enabled=%s'
            % (self.esp32_ip, self.esp32_port, self.enabled))
        return SetParametersResult(successful=True)

    def on_cmd_vel(self, msg):
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        left = linear - angular * self.wheel_base_m / 2.0
        right = linear + angular * self.wheel_base_m / 2.0

        if self.swap_wheels:
            left, right = right, left
        if self.invert_left:
            left = -left
        if self.invert_right:
            right = -right

        self.target_left = self.limit_and_compensate(left)
        self.target_right = self.limit_and_compensate(right)
        self.last_cmd_time = self.get_clock().now()

    def limit_and_compensate(self, speed):
        if not math.isfinite(speed):
            return 0.0
        if abs(speed) < 1e-6:
            return 0.0

        if self.max_wheel_speed_mps > 0.0:
            speed = max(-self.max_wheel_speed_mps, min(self.max_wheel_speed_mps, speed))
        if abs(speed) < self.min_effective_speed_mps:
            speed = math.copysign(self.min_effective_speed_mps, speed)
        return speed

    def on_timer(self):
        left = self.target_left
        right = self.target_right

        if not self.enabled or self.last_cmd_time is None:
            left = 0.0
            right = 0.0
        elif (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout_sec:
            left = 0.0
            right = 0.0

        self.send_wheel_speeds(left, right)

    def send_wheel_speeds(self, left, right):
        payload = 'V %.3f %.3f\n' % (left, right)
        try:
            self.sock.sendto(payload.encode('ascii'), (self.esp32_ip, self.esp32_port))
        except OSError as exc:
            self.get_logger().warning('UDP send failed: %s' % exc)
            return

        current = (round(left, 3), round(right, 3))
        if current != self.last_sent:
            self.get_logger().info('sent %s' % payload.strip())
            self.last_sent = current

    def destroy_node(self):
        try:
            self.send_wheel_speeds(0.0, 0.0)
            self.sock.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveUdp()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
