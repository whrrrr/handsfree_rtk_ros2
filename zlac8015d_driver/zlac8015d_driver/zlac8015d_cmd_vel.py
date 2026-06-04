import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
import serial


REG_WATCHDOG_MS = 0x2000
REG_RUN_MODE = 0x200D
REG_CONTROL = 0x200E
REG_ASYNC_SYNC = 0x200F
REG_TARGET_SPEED_L = 0x2088

MODE_SPEED = 3
CONTROL_STOP = 0x07
CONTROL_ENABLE = 0x08


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def add_crc(payload):
    crc = crc16_modbus(payload)
    return payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def parse_driver_ids(value):
    if isinstance(value, str):
        parts = [p.strip() for p in value.split(',') if p.strip()]
        return [int(p, 0) for p in parts]
    if isinstance(value, (list, tuple)):
        return [int(v) for v in value]
    return [int(value)]


def int16_to_u16(value):
    value = int(value)
    if value < 0:
        value = (1 << 16) + value
    return value & 0xFFFF


class Zlac8015dBus:
    def __init__(self, port, baudrate, timeout=0.08, retries=1):
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
            write_timeout=timeout,
        )
        self.lock = threading.Lock()
        self.retries = max(1, int(retries))

    def close(self):
        self.serial.close()

    def write_register(self, slave_id, address, value):
        payload = bytes((
            slave_id & 0xFF,
            0x06,
            (address >> 8) & 0xFF,
            address & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ))
        return self._request(add_crc(payload), expected_len=8)

    def write_registers(self, slave_id, start_address, values):
        count = len(values)
        body = bytearray((
            slave_id & 0xFF,
            0x10,
            (start_address >> 8) & 0xFF,
            start_address & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
            count * 2,
        ))
        for value in values:
            value &= 0xFFFF
            body.append((value >> 8) & 0xFF)
            body.append(value & 0xFF)
        return self._request(add_crc(bytes(body)), expected_len=8)

    def read_registers(self, slave_id, start_address, count):
        payload = bytes((
            slave_id & 0xFF,
            0x03,
            (start_address >> 8) & 0xFF,
            start_address & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF,
        ))
        reply = self._request(add_crc(payload), expected_len=5 + count * 2)
        values = []
        for idx in range(count):
            off = 3 + idx * 2
            values.append((reply[off] << 8) | reply[off + 1])
        return values

    def _request(self, frame, expected_len):
        last_reply = b''
        for attempt in range(self.retries):
            with self.lock:
                self.serial.reset_input_buffer()
                self.serial.write(frame)
                self.serial.flush()
                reply = self.serial.read(expected_len)
            last_reply = reply
            if len(reply) == expected_len:
                break
            if attempt + 1 < self.retries:
                time.sleep(0.03)
        if len(reply) != expected_len:
            raise TimeoutError('short Modbus reply: %s' % last_reply.hex(' '))
        if crc16_modbus(reply[:-2]) != (reply[-2] | (reply[-1] << 8)):
            raise ValueError('bad Modbus CRC: %s' % reply.hex(' '))
        if reply[1] & 0x80:
            raise RuntimeError('Modbus exception: %s' % reply.hex(' '))
        return reply


class Zlac8015dCmdVel(Node):
    def __init__(self):
        super().__init__('zlac8015d_cmd_vel')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('driver_ids', '1,2')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_radius_m', 0.04)
        self.declare_parameter('wheel_base_m', 0.355)
        self.declare_parameter('max_rpm', 30.0)
        self.declare_parameter('send_rate_hz', 10.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('watchdog_ms', 500)
        self.declare_parameter('enabled', True)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('swap_left_right', False)
        self.declare_parameter('id1_left_sign', 1)
        self.declare_parameter('id1_right_sign', 1)
        self.declare_parameter('id2_left_sign', 1)
        self.declare_parameter('id2_right_sign', 1)

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.driver_ids = parse_driver_ids(self.get_parameter('driver_ids').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.wheel_radius_m = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base_m = float(self.get_parameter('wheel_base_m').value)
        self.max_rpm = abs(float(self.get_parameter('max_rpm').value))
        self.send_rate_hz = float(self.get_parameter('send_rate_hz').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.watchdog_ms = int(self.get_parameter('watchdog_ms').value)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.invert_linear = bool(self.get_parameter('invert_linear').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.swap_left_right = bool(self.get_parameter('swap_left_right').value)

        self.signs = self._load_signs()
        self.bus = None
        self.target_left_rpm = 0
        self.target_right_rpm = 0
        self.last_cmd_time = None
        self.last_sent = None

        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_timer(1.0 / max(self.send_rate_hz, 0.5), self.on_timer)

        self.open_bus()
        self.configure_drivers()
        self.get_logger().info(
            'ZLAC8015D ready: port=%s baud=%d ids=%s topic=%s radius=%.3fm base=%.3fm max=%.1frpm'
            % (self.port, self.baudrate, self.driver_ids, self.cmd_vel_topic,
               self.wheel_radius_m, self.wheel_base_m, self.max_rpm)
        )

    def _load_signs(self):
        signs = {}
        for driver_id in (1, 2):
            left = int(self.get_parameter('id%d_left_sign' % driver_id).value)
            right = int(self.get_parameter('id%d_right_sign' % driver_id).value)
            signs[driver_id] = (1 if left >= 0 else -1, 1 if right >= 0 else -1)
        return signs

    def open_bus(self):
        try:
            self.bus = Zlac8015dBus(self.port, self.baudrate)
        except Exception as exc:
            self.get_logger().error('open RS485 port failed: %s' % exc)
            raise

    def configure_drivers(self):
        for driver_id in self.driver_ids:
            try:
                if self.watchdog_ms > 0:
                    self.bus.write_register(driver_id, REG_WATCHDOG_MS, self.watchdog_ms)
                self.bus.write_register(driver_id, REG_RUN_MODE, MODE_SPEED)
                self.bus.write_register(driver_id, REG_ASYNC_SYNC, 0)
                self.bus.write_registers(driver_id, REG_TARGET_SPEED_L, [0, 0])
                self.bus.write_register(driver_id, REG_CONTROL, CONTROL_ENABLE)
                self.get_logger().info('driver id %d configured for speed mode' % driver_id)
            except Exception as exc:
                self.get_logger().warning('configure driver id %d failed: %s' % (driver_id, exc))

    def on_set_parameters(self, params):
        for param in params:
            if param.name == 'enabled':
                self.enabled = bool(param.value)
            elif param.name == 'max_rpm':
                self.max_rpm = abs(float(param.value))
            elif param.name == 'wheel_radius_m':
                self.wheel_radius_m = float(param.value)
            elif param.name == 'wheel_base_m':
                self.wheel_base_m = float(param.value)
            elif param.name == 'cmd_timeout_sec':
                self.cmd_timeout_sec = float(param.value)
            elif param.name == 'invert_linear':
                self.invert_linear = bool(param.value)
            elif param.name == 'invert_left':
                self.invert_left = bool(param.value)
            elif param.name == 'invert_right':
                self.invert_right = bool(param.value)
            elif param.name == 'swap_left_right':
                self.swap_left_right = bool(param.value)
        return SetParametersResult(successful=True)

    def on_cmd_vel(self, msg):
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if self.invert_linear:
            linear = -linear

        left_mps = linear - angular * self.wheel_base_m / 2.0
        right_mps = linear + angular * self.wheel_base_m / 2.0

        if self.swap_left_right:
            left_mps, right_mps = right_mps, left_mps
        if self.invert_left:
            left_mps = -left_mps
        if self.invert_right:
            right_mps = -right_mps

        self.target_left_rpm = self.mps_to_rpm(left_mps)
        self.target_right_rpm = self.mps_to_rpm(right_mps)
        self.last_cmd_time = self.get_clock().now()

    def mps_to_rpm(self, speed_mps):
        if not math.isfinite(speed_mps) or self.wheel_radius_m <= 0.0:
            return 0
        rpm = speed_mps * 60.0 / (2.0 * math.pi * self.wheel_radius_m)
        if self.max_rpm > 0.0:
            rpm = max(-self.max_rpm, min(self.max_rpm, rpm))
        return int(round(rpm))

    def on_timer(self):
        left = self.target_left_rpm
        right = self.target_right_rpm

        if not self.enabled or self.last_cmd_time is None:
            left = 0
            right = 0
        else:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout_sec:
                left = 0
                right = 0

        self.send_all(left, right)

    def send_all(self, left_rpm, right_rpm):
        if self.bus is None:
            return
        current = (left_rpm, right_rpm, self.enabled)
        for driver_id in self.driver_ids:
            left_sign, right_sign = self.signs.get(driver_id, (1, 1))
            values = [
                int16_to_u16(left_rpm * left_sign),
                int16_to_u16(right_rpm * right_sign),
            ]
            try:
                self.bus.write_registers(driver_id, REG_TARGET_SPEED_L, values)
            except Exception as exc:
                self.get_logger().warning(
                    'send speed to id %d failed: %s' % (driver_id, exc),
                    throttle_duration_sec=1.0,
                )
        if current != self.last_sent:
            self.get_logger().info('target rpm left=%d right=%d' % (left_rpm, right_rpm))
            self.last_sent = current

    def stop_all(self):
        if self.bus is None:
            return
        for driver_id in self.driver_ids:
            try:
                self.bus.write_registers(driver_id, REG_TARGET_SPEED_L, [0, 0])
                time.sleep(0.02)
                self.bus.write_register(driver_id, REG_CONTROL, CONTROL_STOP)
            except Exception as exc:
                self.get_logger().warning('stop id %d failed: %s' % (driver_id, exc))

    def destroy_node(self):
        try:
            self.stop_all()
            if self.bus is not None:
                self.bus.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Zlac8015dCmdVel()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
