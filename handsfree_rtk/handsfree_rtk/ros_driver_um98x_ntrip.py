import base64
import errno
import select
import socket
import threading
import time

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from handsfree_rtk.rtk_common import (
    FIX_TYPE_MAPPING,
    GNRMC_MODE_MAPPING,
    parse_nmea_sentence,
    sleep_until_stopped,
    verify_nmea_checksum,
)


class NtripGnssNode(Node):
    def __init__(self):
        super().__init__('handsfree_rtk_ntrip')

        self.declare_parameter('port', '/dev/HFRobotRTK')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('ntrip_server', '120.253.239.161')
        self.declare_parameter('ntrip_port', 8002)
        self.declare_parameter('ntrip_username', 'ctea952')
        self.declare_parameter('ntrip_password', 'cm286070')
        self.declare_parameter('ntrip_mountpoint', 'RTCM33_GRCE')
        self.declare_parameter('gga_period', 3.0)
        self.declare_parameter('serial_timeout', 1.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.ntrip_server = self.get_parameter('ntrip_server').value
        self.ntrip_port = int(self.get_parameter('ntrip_port').value)
        self.ntrip_username = self.get_parameter('ntrip_username').value
        self.ntrip_password = self.get_parameter('ntrip_password').value
        self.ntrip_mountpoint = self.get_parameter('ntrip_mountpoint').value
        self.gga_period = float(self.get_parameter('gga_period').value)
        self.serial_timeout = float(self.get_parameter('serial_timeout').value)

        self.rtk_raw_publisher = self.create_publisher(String, 'handsfree/rtk/raw', 10)
        self.gnss_publisher = self.create_publisher(NavSatFix, 'handsfree/rtk/gnss', 10)
        self.speed_publisher = self.create_publisher(Float64, 'handsfree/rtk/speed', 10)
        self.cog_pub = self.create_publisher(Float64, 'handsfree/rtk/cog', 10)
        self.heading_pub = self.create_publisher(Float64, 'handsfree/rtk/heading', 10)

        self.serial_port = None
        self.ntrip_sock = None
        self.latest_gga = None
        self.have_gga = False
        self.stop_evt = threading.Event()

        self.threads = [
            threading.Thread(target=self._serial_loop, name='rtk_serial'),
            threading.Thread(target=self._ntrip_connect_loop, name='ntrip_connect'),
            threading.Thread(target=self._loop_send_gga, name='gga_tx'),
            threading.Thread(target=self._loop_recv_rtcm, name='rtcm_rx'),
        ]
        for thread in self.threads:
            thread.daemon = True

    def start(self):
        self.get_logger().info(
            'NTRIP server: %s:%d / %s'
            % (self.ntrip_server, self.ntrip_port, self.ntrip_mountpoint))
        for thread in self.threads:
            thread.start()

    def stop(self):
        self.stop_evt.set()
        self._close_all()
        for thread in self.threads:
            if thread.is_alive():
                thread.join(timeout=1.0)

    def _serial_loop(self):
        while rclpy.ok() and not self.stop_evt.is_set():
            try:
                self._open_serial_blocking()
                while rclpy.ok() and not self.stop_evt.is_set():
                    line = self._readline()
                    if line:
                        self._handle_nmea_line(line)
                    time.sleep(0.005)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().warning('Serial error: %s, try reopen...' % exc)
            except Exception as exc:
                self.get_logger().error('Unknown serial error: %s' % exc)
            finally:
                self._close_serial()
                sleep_until_stopped(self.stop_evt, 0.5)

    def _open_serial_blocking(self):
        backoff = 0.5
        while rclpy.ok() and not self.stop_evt.is_set():
            try:
                self.serial_port = serial.Serial(
                    self.port, self.baudrate, timeout=self.serial_timeout)
                self.get_logger().info('Opened serial %s @ %d' % (self.port, self.baudrate))
                return
            except Exception as exc:
                self.get_logger().warning('Open serial failed: %s' % exc)
                sleep_until_stopped(self.stop_evt, backoff)
                backoff = min(5.0, backoff * 1.5)

    def _readline(self):
        if self.serial_port is None:
            return None
        raw = self.serial_port.readline()
        if not raw:
            return None
        if isinstance(raw, bytes):
            return raw.decode('utf-8', errors='ignore').strip()
        return raw.strip()

    def _ntrip_connect_loop(self):
        while rclpy.ok() and not self.stop_evt.is_set():
            if self.ntrip_sock or not self.have_gga:
                sleep_until_stopped(self.stop_evt, 0.2)
                continue
            self._connect_ntrip()
            sleep_until_stopped(self.stop_evt, 1.0)

    def _connect_ntrip(self):
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((self.ntrip_server, self.ntrip_port))
            auth = base64.b64encode(
                ('%s:%s' % (self.ntrip_username, self.ntrip_password)).encode('utf-8')
            ).decode('utf-8')
            request = (
                'GET /%s HTTP/1.0\r\n'
                'User-Agent: NTRIP ntrip_client\r\n'
                'Accept: */*\r\n'
                'Connection: close\r\n'
                'Authorization: Basic %s\r\n'
                '\r\n'
            ) % (self.ntrip_mountpoint, auth)
            sock.send(request.encode('utf-8'))
            sock.settimeout(0.0)
            self.ntrip_sock = sock
            self.get_logger().info('NTRIP socket connected, waiting for RTCM data.')
        except Exception as exc:
            self.get_logger().error('Failed to connect to NTRIP server: %s' % exc)
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

    def _loop_send_gga(self):
        period = max(0.2, float(self.gga_period))
        next_time = time.time()
        while rclpy.ok() and not self.stop_evt.is_set():
            now = time.time()
            if now < next_time:
                sleep_until_stopped(self.stop_evt, next_time - now)
                continue
            next_time = now + period

            if not self.ntrip_sock or not self.latest_gga:
                continue
            payload = (self.latest_gga + '\r\n').encode('utf-8')
            try:
                _, writable, _ = select.select([], [self.ntrip_sock], [], 0)
                if writable:
                    self.ntrip_sock.send(payload)
            except OSError as exc:
                if getattr(exc, 'errno', None) in (errno.EPIPE, errno.ENOTCONN):
                    self.get_logger().warning('NTRIP send broken pipe.')
                    self._safe_close_sock()
            except Exception as exc:
                self.get_logger().debug('Send GGA failed: %s' % exc)

    def _loop_recv_rtcm(self):
        while rclpy.ok() and not self.stop_evt.is_set():
            if not self.ntrip_sock or not self.serial_port:
                sleep_until_stopped(self.stop_evt, 0.2)
                continue
            try:
                readable, _, _ = select.select([self.ntrip_sock], [], [], 0.2)
                if not readable:
                    continue
                data = self.ntrip_sock.recv(4096)
                if not data:
                    self._safe_close_sock()
                    continue
                if b'ICY 200 OK' in data:
                    self.get_logger().info('Connected to NTRIP server (ICY 200 OK).')
                    continue
                self.serial_port.write(data)
            except Exception:
                pass

    def _handle_nmea_line(self, line):
        self.rtk_raw_publisher.publish(String(data=line))

        if not line or (line[0] != '$' and not line.startswith('#UNIHEADINGA')):
            return

        if line.startswith(('$GPGGA', '$GNGGA')) and verify_nmea_checksum(line):
            parts = line.split(',')
            has_latlon = len(parts) > 5 and parts[2] != '' and parts[4] != ''
            if has_latlon:
                self.latest_gga = line
                self.have_gga = True

        info = parse_nmea_sentence(line)
        if not info:
            return

        if info.get('type') == 'GNGGA':
            self._publish_gngga_data(info)
        elif info.get('type') == 'GNRMC':
            self._publish_gnrmc_data(info)
        elif info.get('type') in ('GNTHS', 'UNIHEADINGA'):
            self._publish_gnths_data(info)

    def _publish_gngga_data(self, gga):
        fix_q = int(gga.get('fix_quality', 0))
        lat = gga.get('latitude')
        lon = gga.get('longitude')
        alt = gga.get('altitude')
        if fix_q == 0 or lat is None or lon is None or alt is None:
            self.get_logger().debug('GGA skipped: no fix or missing fields.')
            return

        status_desc, status_value, cov_m = FIX_TYPE_MAPPING.get(
            fix_q, ('Unknown', NavSatStatus.STATUS_NO_FIX, 10000.0))
        nav = NavSatFix()
        nav.header.stamp = self.get_clock().now().to_msg()
        nav.header.frame_id = self.frame_id
        nav.status.status = status_value
        nav.status.service = NavSatStatus.SERVICE_GPS
        nav.latitude = float(lat)
        nav.longitude = float(lon)
        nav.altitude = float(alt)
        var = cov_m ** 2
        nav.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
        nav.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gnss_publisher.publish(nav)
        self.get_logger().info(
            'GGA: fix=%d (%s), lat=%.8f lon=%.8f alt=%.3f'
            % (fix_q, status_desc, nav.latitude, nav.longitude, nav.altitude))

    def _publish_gnrmc_data(self, rmc):
        rmc_status = rmc.get('status', 'V')
        speed_mps = rmc.get('speed_mps', 0.0)
        cog_deg = rmc.get('cog_deg', 0.0)
        if rmc_status == 'V':
            self.get_logger().debug('RMC skipped: invalid.')
            return
        self.speed_publisher.publish(Float64(data=speed_mps))
        self.cog_pub.publish(Float64(data=cog_deg))
        self.get_logger().info(
            'RMC: status=%s, speed=%.3f m/s, COG=%.2f'
            % (GNRMC_MODE_MAPPING.get(rmc_status, 'Unknown'), speed_mps, cog_deg))

    def _publish_gnths_data(self, ths):
        heading = ths.get('heading')
        if heading is not None and ths.get('valid', False):
            self.heading_pub.publish(Float64(data=heading))
            self.get_logger().info('%s: heading=%.3f' % (ths.get('type', 'HEADING'), heading))

    def _close_serial(self):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
        except Exception:
            pass
        finally:
            self.serial_port = None

    def _safe_close_sock(self):
        try:
            if self.ntrip_sock:
                try:
                    self.ntrip_sock.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                self.ntrip_sock.close()
        except Exception:
            pass
        finally:
            self.ntrip_sock = None

    def _close_all(self):
        self._safe_close_sock()
        self._close_serial()


def main(args=None):
    rclpy.init(args=args)
    node = NtripGnssNode()
    node.start()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
