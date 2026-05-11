import threading
import time

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from handsfree_rtk.rtk_common import FIX_TYPE_MAPPING, GNRMC_MODE_MAPPING, parse_nmea_sentence


class RTKTagDriver(Node):
    def __init__(self):
        super().__init__('handsfree_rtk')

        self.declare_parameter('port', '/dev/HFRobotRTK')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('frame_id', 'gps')

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.timeout = float(self.get_parameter('timeout').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.raw_pub = self.create_publisher(String, 'handsfree/rtk/raw', 10)
        self.gnss_pub = self.create_publisher(NavSatFix, 'handsfree/rtk/gnss', 10)
        self.speed_pub = self.create_publisher(Float64, 'handsfree/rtk/speed', 10)
        self.cog_pub = self.create_publisher(Float64, 'handsfree/rtk/cog', 10)
        self.heading_pub = self.create_publisher(Float64, 'handsfree/rtk/heading', 10)

        self._stop_event = threading.Event()
        self._read_thread = threading.Thread(target=self._run_serial_loop, name='nmea_reader')
        self._read_thread.daemon = True
        self._serial = None

    def start(self):
        self._read_thread.start()

    def stop(self):
        self._stop_event.set()
        if self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        self._cleanup_serial()

    def _run_serial_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                self.get_logger().info('Trying to open serial: %s @ %d' % (self.port, self.baudrate))
                self._serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
                self.get_logger().info('Serial opened: %s' % self.port)
                self._read_loop()
            except serial.SerialException as exc:
                self.get_logger().error('Serial error: %s. Retry in 1s...' % exc)
            except Exception as exc:
                self.get_logger().error('Unexpected serial loop error: %s. Retry in 1s...' % exc)
            finally:
                self._cleanup_serial()
            time.sleep(1.0)

    def _read_loop(self):
        while rclpy.ok() and not self._stop_event.is_set():
            line = self._serial.readline()
            if not line:
                time.sleep(0.001)
                continue
            if isinstance(line, bytes):
                line = line.decode('utf-8', 'ignore')
            self._handle_line(line.strip())

    def _handle_line(self, line):
        self.raw_pub.publish(String(data=line))

        if line.startswith(('$GNGGA', '$GPGGA', '$GNRMC', '$GPRMC', '$GNTHS', '$GPTHS')):
            self.get_logger().info(line)

        parsed = parse_nmea_sentence(line)
        if not parsed:
            return

        if parsed['type'] == 'GNGGA':
            self._publish_gga(parsed)
        elif parsed['type'] == 'GNRMC':
            self._publish_rmc(parsed)
        elif parsed['type'] == 'GNTHS':
            self._publish_ths(parsed)

    def _publish_gga(self, parsed):
        lat = parsed.get('latitude')
        lon = parsed.get('longitude')
        alt = parsed.get('altitude')
        fix_q = parsed.get('fix_quality', 0)
        if lat is None or lon is None:
            return

        status_str, status_value, cov_m = FIX_TYPE_MAPPING.get(
            fix_q, ('Unknown', NavSatStatus.STATUS_NO_FIX, 10000.0))

        navsat = NavSatFix()
        navsat.header.stamp = self.get_clock().now().to_msg()
        navsat.header.frame_id = self.frame_id
        navsat.status.status = status_value
        navsat.status.service = NavSatStatus.SERVICE_GPS
        navsat.latitude = lat
        navsat.longitude = lon
        navsat.altitude = alt if alt is not None else 0.0
        var = cov_m ** 2
        navsat.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gnss_pub.publish(navsat)

        self.get_logger().info(
            'GGA: fix=%d (%s), lat=%.8f lon=%.8f alt=%.3f'
            % (fix_q, status_str, navsat.latitude, navsat.longitude, navsat.altitude))

    def _publish_rmc(self, parsed):
        status = parsed.get('status', 'V')
        speed_mps = parsed.get('speed_mps', 0.0)
        cog_deg = parsed.get('cog_deg', 0.0)
        self.get_logger().info(
            'RMC: status=%s, speed=%.3f m/s (%.3f kn), COG=%.2f'
            % (GNRMC_MODE_MAPPING.get(status, 'Unknown'), speed_mps,
               parsed.get('raw_speed_knots', 0.0), cog_deg))
        if status != 'V':
            self.speed_pub.publish(Float64(data=speed_mps))
            self.cog_pub.publish(Float64(data=cog_deg))

    def _publish_ths(self, parsed):
        heading = parsed.get('heading')
        if heading is not None and parsed.get('valid', False):
            self.heading_pub.publish(Float64(data=heading))
            self.get_logger().info('THS: heading=%.3f' % heading)

    def _cleanup_serial(self):
        try:
            if self._serial and self._serial.is_open:
                port_name = self._serial.port
                self._serial.close()
                self.get_logger().info('Serial closed: %s' % port_name)
        except Exception as exc:
            self.get_logger().warning('Serial close warning: %s' % exc)
        finally:
            self._serial = None


def main(args=None):
    rclpy.init(args=args)
    node = RTKTagDriver()
    node.start()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
