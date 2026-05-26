#!/usr/bin/env python3
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

try:
    import yaml
except ImportError:
    yaml = None


def _parent_dirs(path):
    current = os.path.abspath(path)
    while True:
        yield current
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent


def _default_waypoint_file():
    for base_dir in _parent_dirs(os.getcwd()):
        candidates = (
            os.path.join(base_dir, 'src', 'gps', 'gps_waypoint_nav', 'config', 'waypoints.yaml'),
            os.path.join(base_dir, 'gps_waypoint_nav', 'config', 'waypoints.yaml'),
            os.path.join(base_dir, 'config', 'waypoints.yaml'),
        )
        for path in candidates:
            if os.path.exists(path):
                return path
    return os.path.join(
        os.path.expanduser('~'),
        'cc_ws', 'tros_ws', 'src', 'gps', 'gps_waypoint_nav', 'config', 'waypoints.yaml')


class RtkHttpBridge(Node):
    def __init__(self):
        super().__init__('rtk_http_bridge')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('api_path', '/api/command')
        self.declare_parameter('gnss_topic', 'handsfree/rtk/gnss')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('follower_node', '/gps_waypoint_follower')
        self.declare_parameter('diff_drive_node', '/diff_drive_udp')
        self.declare_parameter('waypoint_file', _default_waypoint_file())
        self.declare_parameter('require_fix_for_capture', True)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.api_path = str(self.get_parameter('api_path').value)
        self.follower_node = str(self.get_parameter('follower_node').value)
        self.diff_drive_node = str(self.get_parameter('diff_drive_node').value)
        self.waypoint_file = str(self.get_parameter('waypoint_file').value)
        self.require_fix_for_capture = bool(self.get_parameter('require_fix_for_capture').value)

        self.latest_fix = None
        self.latest_fix_ts = 0.0
        self._httpd = None
        self._http_thread = None
        self._lock = threading.Lock()

        self.create_subscription(
            NavSatFix,
            str(self.get_parameter('gnss_topic').value),
            self._on_fix,
            20,
        )
        self.cmd_pub = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self._set_param_client = self.create_client(
            SetParameters, f'{self.follower_node.rstrip("/")}/set_parameters')
        self._set_diff_drive_param_client = self.create_client(
            SetParameters, f'{self.diff_drive_node.rstrip("/")}/set_parameters')

        self._start_http_server()

    def _on_fix(self, msg: NavSatFix):
        with self._lock:
            self.latest_fix = msg
            self.latest_fix_ts = time.time()

    def _start_http_server(self):
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                bridge.get_logger().info('HTTP ' + fmt % args)

            def _send_json(self, status_code, data):
                body = json.dumps(data, ensure_ascii=False).encode('utf-8')
                self.send_response(status_code)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type,Authorization')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_OPTIONS(self):
                self.send_response(204)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET,POST,OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type,Authorization')
                self.end_headers()

            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == '/healthz':
                    self._send_json(200, {'ok': True, 'node': bridge.get_name()})
                    return
                if parsed.path != bridge.api_path:
                    self._send_json(404, {'ok': False, 'error': 'not_found'})
                    return
                query = parse_qs(parsed.query)
                cmd = (query.get('cmd') or query.get('action') or [''])[0]
                payload = {'cmd': cmd}
                for key, values in query.items():
                    if key in ('cmd', 'action'):
                        continue
                    payload[key] = values[0] if len(values) == 1 else values
                result = bridge.handle_command(payload)
                self._send_json(200 if result.get('ok') else 400, result)

            def do_POST(self):
                parsed = urlparse(self.path)
                if parsed.path != bridge.api_path:
                    self._send_json(404, {'ok': False, 'error': 'not_found'})
                    return

                length = int(self.headers.get('Content-Length', '0'))
                raw = self.rfile.read(length).decode('utf-8', errors='ignore') if length > 0 else ''
                ctype = (self.headers.get('Content-Type') or '').lower()

                try:
                    if 'application/json' in ctype:
                        payload = json.loads(raw) if raw.strip() else {}
                    else:
                        # text/plain, form, or unknown payloads: try JSON first then plain cmd text.
                        try:
                            payload = json.loads(raw)
                        except Exception:
                            payload = {'cmd': raw.strip()}
                except Exception as exc:
                    self._send_json(400, {'ok': False, 'error': f'bad_payload: {exc}'})
                    return

                result = bridge.handle_command(payload)
                self._send_json(200 if result.get('ok') else 400, result)

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()
        self.get_logger().info(
            f'HTTP server started: http://{self.host}:{self.port}{self.api_path} (health: /healthz)')

    def _write_waypoint_yaml(self, waypoint_latitudes, waypoint_longitudes, waypoint_names):
        if yaml is None:
            return False, 'pyyaml not installed'
        data = {
            'gps_waypoint_follower': {
                'ros__parameters': {
                    'enabled': False,
                    'gnss_topic': 'handsfree/rtk/gnss',
                    'cog_topic': 'handsfree/rtk/cog',
                    'cmd_vel_topic': '/cmd_vel',
                    'waypoint_latitudes': waypoint_latitudes,
                    'waypoint_longitudes': waypoint_longitudes,
                    'waypoint_names': waypoint_names,
                }
            }
        }
        path = os.path.abspath(self.waypoint_file)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
        return True, path

    def _load_waypoint_yaml(self):
        if yaml is None:
            return None, 'pyyaml not installed'
        path = os.path.abspath(self.waypoint_file)
        if not os.path.exists(path):
            return {'gps_waypoint_follower': {'ros__parameters': {}}}, None
        with open(path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        data.setdefault('gps_waypoint_follower', {})
        data['gps_waypoint_follower'].setdefault('ros__parameters', {})
        return data, None

    def _set_nav_enabled(self, enabled):
        if not self._set_param_client.wait_for_service(timeout_sec=1.0):
            return False, 'set_parameters service unavailable'

        req = SetParameters.Request()
        req.parameters = [
            ParameterMsg(
                name='enabled',
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(enabled)),
            )
        ]
        future = self._set_param_client.call_async(req)
        done = threading.Event()

        def _done(_future):
            done.set()

        future.add_done_callback(_done)
        if not done.wait(timeout=2.0):
            return False, 'set enabled timeout'

        try:
            response = future.result()
            if not response.results:
                return False, 'empty set_parameters response'
            if not response.results[0].successful:
                reason = response.results[0].reason or 'rejected'
                return False, f'set enabled failed: {reason}'
            return True, 'ok'
        except Exception as exc:
            return False, f'set enabled exception: {exc}'

    def _set_diff_drive_target(self, ip, port=None):
        if not ip or not str(ip).strip():
            return False, 'empty control board ip'
        ip = str(ip).strip()
        try:
            # Basic validation for IPv4 literal; keep hostnames unsupported for clarity.
            socket.inet_aton(ip)
        except OSError:
            return False, 'invalid IPv4 address'

        if not self._set_diff_drive_param_client.wait_for_service(timeout_sec=1.0):
            return False, 'diff_drive_udp set_parameters service unavailable'

        req = SetParameters.Request()
        req.parameters = [
            ParameterMsg(
                name='esp32_ip',
                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=ip),
            )
        ]
        if port is not None:
            req.parameters.append(
                ParameterMsg(
                    name='esp32_port',
                    value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=int(port)),
                )
            )

        future = self._set_diff_drive_param_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout=2.0):
            return False, 'set control board target timeout'

        try:
            response = future.result()
            if not response.results:
                return False, 'empty set_parameters response'
            for result in response.results:
                if not result.successful:
                    return False, f'set control board target failed: {result.reason or "rejected"}'
            if port is None:
                return True, f'updated control board ip to {ip}'
            return True, f'updated control board target to {ip}:{int(port)}'
        except Exception as exc:
            return False, f'set control board target exception: {exc}'

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _publish_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _capture_waypoint(self, name=''):
        with self._lock:
            fix = self.latest_fix
            ts = self.latest_fix_ts
        if fix is None:
            return False, 'no GNSS fix received yet'
        if self.require_fix_for_capture and fix.status.status == NavSatStatus.STATUS_NO_FIX:
            return False, 'GNSS status NO_FIX'
        if not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
            return False, 'invalid lat/lon'
        if time.time() - ts > 5.0:
            return False, 'latest fix too old (>5s)'

        data, err = self._load_waypoint_yaml()
        if err:
            return False, err
        params = data['gps_waypoint_follower']['ros__parameters']
        latitudes = list(params.get('waypoint_latitudes') or [])
        longitudes = list(params.get('waypoint_longitudes') or [])
        names = list(params.get('waypoint_names') or [])
        wp_index = len(latitudes) + 1
        wp_name = (name or f'wp_{wp_index}').strip()

        latitudes.append(float(fix.latitude))
        longitudes.append(float(fix.longitude))
        names.append(wp_name)

        ok, msg = self._write_waypoint_yaml(latitudes, longitudes, names)
        if not ok:
            return False, msg
        return True, f'saved {wp_name} ({fix.latitude:.8f}, {fix.longitude:.8f}) count={len(latitudes)}'

    def _clear_waypoints(self):
        ok, msg = self._write_waypoint_yaml([], [], [])
        if not ok:
            return False, msg
        return True, 'cleared waypoints'

    def handle_command(self, payload):
        if isinstance(payload, str):
            payload = {'cmd': payload}
        payload = payload or {}
        cmd = str(payload.get('cmd') or payload.get('action') or '').strip().lower()

        self.get_logger().info(f'HTTP command: {payload}')

        if cmd in ('ping', 'health'):
            return {'ok': True, 'message': 'pong'}

        if cmd in ('enable_nav', 'nav_enable'):
            enabled = bool(payload.get('value', True))
            ok, msg = self._set_nav_enabled(enabled)
            return {'ok': ok, 'message': msg, 'enabled': enabled}

        if cmd in ('start_follow', 'start'):
            ok, msg = self._set_nav_enabled(True)
            return {'ok': ok, 'message': msg, 'enabled': True}

        if cmd in ('stop_follow', 'disable_nav'):
            ok, msg = self._set_nav_enabled(False)
            self._publish_stop()
            return {'ok': ok, 'message': msg, 'enabled': False}

        if cmd in ('stop', 'estop'):
            ok, msg = self._set_nav_enabled(False)
            self._publish_stop()
            return {'ok': ok, 'message': f'{msg}; cmd_vel zero published', 'enabled': False}

        if cmd in ('capture_waypoint', 'capture'):
            ok, msg = self._capture_waypoint(str(payload.get('name', '')).strip())
            return {'ok': ok, 'message': msg}

        if cmd in ('clear_waypoints', 'clear'):
            ok, msg = self._clear_waypoints()
            return {'ok': ok, 'message': msg}

        if cmd in ('cmd_vel', 'teleop'):
            linear_x = float(payload.get('linear_x', 0.0))
            angular_z = float(payload.get('angular_z', 0.0))
            self._publish_cmd_vel(linear_x, angular_z)
            return {'ok': True, 'message': 'cmd_vel published', 'linear_x': linear_x, 'angular_z': angular_z}

        if cmd in ('set_control_ip', 'set_esp32_ip', 'set_diff_drive_target'):
            ip = payload.get('ip') or payload.get('esp32_ip') or ''
            port = payload.get('port', None)
            ok, msg = self._set_diff_drive_target(ip, port)
            return {'ok': ok, 'message': msg, 'ip': ip, 'port': port}

        return {'ok': False, 'error': f'unknown cmd: {cmd}'}

    def close(self):
        if self._httpd:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None


def main(args=None):
    rclpy.init(args=args)
    node = RtkHttpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
