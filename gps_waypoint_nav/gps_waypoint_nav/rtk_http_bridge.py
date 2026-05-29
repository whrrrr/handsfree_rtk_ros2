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
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

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
        self.declare_parameter('heading_topic', 'handsfree/rtk/heading')
        self.declare_parameter('cog_topic', 'handsfree/rtk/cog')
        self.declare_parameter('status_topic', '/gps_waypoint_follower/status')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('follower_node', '/gps_waypoint_follower')
        self.declare_parameter('diff_drive_node', '/diff_drive_udp')
        self.declare_parameter('waypoint_file', _default_waypoint_file())
        self.declare_parameter('require_fix_for_capture', True)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.api_path = str(self.get_parameter('api_path').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.follower_node = str(self.get_parameter('follower_node').value)
        self.diff_drive_node = str(self.get_parameter('diff_drive_node').value)
        self.waypoint_file = str(self.get_parameter('waypoint_file').value)
        self.require_fix_for_capture = bool(self.get_parameter('require_fix_for_capture').value)

        self.latest_fix = None
        self.latest_fix_ts = 0.0
        self.latest_heading = None
        self.latest_heading_ts = 0.0
        self.latest_cog = None
        self.latest_cog_ts = 0.0
        self.latest_status = ''
        self.latest_status_ts = 0.0
        self.latest_cmd_vel = {'linear_x': 0.0, 'angular_z': 0.0}
        self.latest_cmd_vel_ts = 0.0
        self._httpd = None
        self._http_thread = None
        self._lock = threading.Lock()

        self.create_subscription(
            NavSatFix,
            str(self.get_parameter('gnss_topic').value),
            self._on_fix,
            20,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter('heading_topic').value),
            self._on_heading,
            20,
        )
        self.create_subscription(
            Float64,
            str(self.get_parameter('cog_topic').value),
            self._on_cog,
            20,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('status_topic').value),
            self._on_status,
            20,
        )
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 20)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._get_param_client = self.create_client(
            GetParameters, f'{self.follower_node.rstrip("/")}/get_parameters')
        self._set_param_client = self.create_client(
            SetParameters, f'{self.follower_node.rstrip("/")}/set_parameters')
        self._set_diff_drive_param_client = self.create_client(
            SetParameters, f'{self.diff_drive_node.rstrip("/")}/set_parameters')

        self._start_http_server()

    def _on_fix(self, msg: NavSatFix):
        with self._lock:
            self.latest_fix = msg
            self.latest_fix_ts = time.time()

    def _on_heading(self, msg: Float64):
        with self._lock:
            self.latest_heading = float(msg.data)
            self.latest_heading_ts = time.time()

    def _on_cog(self, msg: Float64):
        with self._lock:
            self.latest_cog = float(msg.data)
            self.latest_cog_ts = time.time()

    def _on_status(self, msg: String):
        with self._lock:
            self.latest_status = str(msg.data)
            self.latest_status_ts = time.time()

    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self.latest_cmd_vel = {
                'linear_x': float(msg.linear.x),
                'angular_z': float(msg.angular.z),
            }
            self.latest_cmd_vel_ts = time.time()

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
                if parsed.path == '/api/state':
                    self._send_json(200, bridge.snapshot_state())
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
                    'heading_topic': 'handsfree/rtk/heading',
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

    def snapshot_state(self):
        now = time.time()
        with self._lock:
            fix = self.latest_fix
            fix_ts = self.latest_fix_ts
            heading = self.latest_heading
            heading_ts = self.latest_heading_ts
            cog = self.latest_cog
            cog_ts = self.latest_cog_ts
            status = self.latest_status
            status_ts = self.latest_status_ts
            cmd_vel = dict(self.latest_cmd_vel)
            cmd_vel_ts = self.latest_cmd_vel_ts

        waypoints = self._waypoint_snapshot()
        state = {
            'ok': True,
            'time': now,
            'gnss': None,
            'heading': {
                'value': heading,
                'age_sec': None if heading_ts <= 0 else now - heading_ts,
            },
            'cog': {
                'value': cog,
                'age_sec': None if cog_ts <= 0 else now - cog_ts,
            },
            'status': {
                'text': status,
                'age_sec': None if status_ts <= 0 else now - status_ts,
                'parsed': self._parse_status(status),
            },
            'cmd_vel': {
                **cmd_vel,
                'age_sec': None if cmd_vel_ts <= 0 else now - cmd_vel_ts,
            },
            'waypoints': waypoints,
        }

        if fix is not None:
            state['gnss'] = {
                'latitude': float(fix.latitude),
                'longitude': float(fix.longitude),
                'altitude': float(fix.altitude),
                'status': int(fix.status.status),
                'service': int(fix.status.service),
                'age_sec': None if fix_ts <= 0 else now - fix_ts,
            }
            self._add_vehicle_xy(state)
        return state

    def _waypoint_snapshot(self):
        data, err = self._load_waypoint_yaml()
        if err:
            return {'ok': False, 'error': err, 'items': []}
        params = data['gps_waypoint_follower']['ros__parameters']
        latitudes = [float(value) for value in (params.get('waypoint_latitudes') or [])]
        longitudes = [float(value) for value in (params.get('waypoint_longitudes') or [])]
        names = [str(value) for value in (params.get('waypoint_names') or [])]
        items = []
        for index, (lat, lon) in enumerate(zip(latitudes, longitudes)):
            items.append({
                'name': names[index] if index < len(names) else f'wp_{index + 1}',
                'latitude': lat,
                'longitude': lon,
            })
        if items:
            origin_lat = items[0]['latitude']
            origin_lon = items[0]['longitude']
            for item in items:
                x, y = self._latlon_to_enu(
                    item['latitude'], item['longitude'], origin_lat, origin_lon)
                item['x'] = x
                item['y'] = y
        return {'ok': True, 'items': items}

    def _add_vehicle_xy(self, state):
        items = state['waypoints']['items']
        if not items:
            return
        x, y = self._latlon_to_enu(
            state['gnss']['latitude'],
            state['gnss']['longitude'],
            items[0]['latitude'],
            items[0]['longitude'],
        )
        state['gnss']['x'] = x
        state['gnss']['y'] = y

    @staticmethod
    def _latlon_to_enu(lat, lon, origin_lat, origin_lon):
        radius_m = 6378137.0
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(origin_lat)
        origin_lon_rad = math.radians(origin_lon)
        x = radius_m * (lon_rad - origin_lon_rad) * math.cos((lat_rad + origin_lat_rad) / 2.0)
        y = radius_m * (lat_rad - origin_lat_rad)
        return x, y

    @staticmethod
    def _enu_to_latlon(x, y, origin_lat, origin_lon):
        radius_m = 6378137.0
        origin_lat_rad = math.radians(origin_lat)
        lat_rad = origin_lat_rad + y / radius_m
        lon_rad = math.radians(origin_lon) + x / (
            radius_m * math.cos((lat_rad + origin_lat_rad) / 2.0))
        return math.degrees(lat_rad), math.degrees(lon_rad)

    @staticmethod
    def _parse_status(text):
        parsed = {}
        if not text:
            return parsed
        if text.startswith('tracking ') or text.startswith('path_tracking '):
            parts = text.split()
            if len(parts) >= 3:
                parsed['mode'] = parts[0]
                parsed['target'] = parts[1]
                parsed['progress'] = parts[2].rstrip(':')
        elif text.startswith('arrived '):
            parsed['mode'] = 'arrived'
            parsed['target'] = text.split(maxsplit=1)[1]
        else:
            parsed['mode'] = text

        fields = {
            'distance=': ('distance_m', 'm'),
            'heading_error=': ('heading_error_deg', 'deg'),
            'v=': ('linear_speed', ''),
            'w=': ('angular_speed', ''),
            'source=': ('source', ''),
            'cross_track=': ('cross_track_m', 'm'),
            'path_s=': ('path_progress_m', 'm'),
            'remaining=': ('remaining_m', 'm'),
            'lookahead=': ('lookahead_m', 'm'),
        }
        for token in text.replace(':', ' ').split():
            for prefix, (name, suffix) in fields.items():
                if token.startswith(prefix):
                    value = token[len(prefix):]
                    if suffix and value.endswith(suffix):
                        value = value[:-len(suffix)]
                    try:
                        parsed[name] = float(value)
                    except ValueError:
                        parsed[name] = value
        return parsed

    def _set_nav_enabled(self, enabled):
        if not self._set_param_client.wait_for_service(timeout_sec=1.0):
            return False, 'set_parameters service unavailable'

        req = SetParameters.Request()
        parameters = []
        if enabled:
            ok, waypoint_params_or_msg = self._waypoint_parameters_from_file()
            if not ok:
                return False, waypoint_params_or_msg
            parameters.extend(waypoint_params_or_msg)

        parameters.append(
            ParameterMsg(
                name='enabled',
                value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(enabled)),
            )
        )
        req.parameters = parameters
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

    def _waypoint_parameters_from_file(self):
        data, err = self._load_waypoint_yaml()
        if err:
            return False, err
        params = data['gps_waypoint_follower']['ros__parameters']
        latitudes = [float(value) for value in (params.get('waypoint_latitudes') or [])]
        longitudes = [float(value) for value in (params.get('waypoint_longitudes') or [])]
        names = [str(value) for value in (params.get('waypoint_names') or [])]

        if len(latitudes) != len(longitudes):
            return False, 'waypoint_latitudes and waypoint_longitudes length mismatch'
        if names and len(names) != len(latitudes):
            return False, 'waypoint_names length mismatch'
        if not names:
            names = [f'wp_{index + 1}' for index in range(len(latitudes))]

        return True, [
            ParameterMsg(
                name='waypoint_latitudes',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    double_array_value=latitudes,
                ),
            ),
            ParameterMsg(
                name='waypoint_longitudes',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                    double_array_value=longitudes,
                ),
            ),
            ParameterMsg(
                name='waypoint_names',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING_ARRAY,
                    string_array_value=names,
                ),
            ),
        ]

    def _get_nav_enabled(self):
        if not self._get_param_client.wait_for_service(timeout_sec=0.2):
            return None, 'follower get_parameters service unavailable'

        req = GetParameters.Request()
        req.names = ['enabled']
        future = self._get_param_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout=0.5):
            return None, 'get enabled timeout'

        try:
            response = future.result()
            if not response.values:
                return None, 'enabled parameter missing'
            value = response.values[0]
            if value.type != ParameterType.PARAMETER_BOOL:
                return None, 'enabled parameter is not bool'
            return bool(value.bool_value), 'ok'
        except Exception as exc:
            return None, f'get enabled exception: {exc}'

    def _set_nav_params(self, params):
        allowed = {
            'arrival_radius_m',
            'max_linear_speed',
            'min_linear_speed',
            'max_angular_speed',
            'heading_kp',
            'lookahead_distance_m',
            'slow_radius_m',
            'large_heading_error_rad',
        }
        parameters = []
        applied = {}
        for name in allowed:
            if name not in params:
                continue
            value = float(params[name])
            if not math.isfinite(value):
                return False, f'invalid {name}', applied
            if name != 'heading_kp' and value < 0.0:
                return False, f'{name} must be >= 0', applied
            parameters.append(
                ParameterMsg(
                    name=name,
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value),
                )
            )
            applied[name] = value

        if not parameters:
            return False, 'no supported nav params provided', applied
        if not self._set_param_client.wait_for_service(timeout_sec=1.0):
            return False, 'set_parameters service unavailable', applied

        req = SetParameters.Request()
        req.parameters = parameters
        future = self._set_param_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout=2.0):
            return False, 'set nav params timeout', applied

        try:
            response = future.result()
            if not response.results:
                return False, 'empty set_parameters response', applied
            for result in response.results:
                if not result.successful:
                    return False, f'set nav params failed: {result.reason or "rejected"}', applied
            return True, 'nav params updated', applied
        except Exception as exc:
            return False, f'set nav params exception: {exc}', applied

    @staticmethod
    def _node_path(info):
        namespace = info.node_namespace or '/'
        name = info.node_name or ''
        if namespace == '/':
            return f'/{name}'
        return f'{namespace.rstrip("/")}/{name}'

    def _cmd_vel_publishers(self):
        infos = self.get_publishers_info_by_topic(self.cmd_vel_topic)
        return sorted(set(self._node_path(info) for info in infos))

    def _check_control_conflicts(self):
        publishers = self._cmd_vel_publishers()
        allowed = {
            f'/{self.get_name()}',
            self.follower_node if self.follower_node.startswith('/') else f'/{self.follower_node}',
        }
        unknown_publishers = [node for node in publishers if node not in allowed]

        nav_enabled, nav_msg = self._get_nav_enabled()
        conflicts = []
        if unknown_publishers:
            conflicts.append('unexpected /cmd_vel publishers: %s' % ', '.join(unknown_publishers))
        if nav_enabled is True:
            conflicts.append('navigation is enabled')

        return {
            'ok': not conflicts,
            'conflicts': conflicts,
            'cmd_vel_topic': self.cmd_vel_topic,
            'cmd_vel_publishers': publishers,
            'nav_enabled': nav_enabled,
            'nav_status': nav_msg,
        }

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

    def _sync_waypoints_to_follower(self):
        if not self._set_param_client.wait_for_service(timeout_sec=0.2):
            return False, 'follower set_parameters service unavailable'
        ok, params_or_msg = self._waypoint_parameters_from_file()
        if not ok:
            return False, params_or_msg
        req = SetParameters.Request()
        req.parameters = params_or_msg
        future = self._set_param_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(timeout=1.0):
            return False, 'sync waypoints timeout'
        try:
            response = future.result()
            failed = [r.reason or 'rejected' for r in response.results if not r.successful]
            if failed:
                return False, '; '.join(failed)
            return True, 'synced to follower'
        except Exception as exc:
            return False, f'sync waypoints exception: {exc}'

    def _load_waypoint_lists(self):
        data, err = self._load_waypoint_yaml()
        if err:
            return None, None, None, err
        params = data['gps_waypoint_follower']['ros__parameters']
        latitudes = [float(value) for value in (params.get('waypoint_latitudes') or [])]
        longitudes = [float(value) for value in (params.get('waypoint_longitudes') or [])]
        names = [str(value) for value in (params.get('waypoint_names') or [])]
        if len(latitudes) != len(longitudes):
            return None, None, None, 'waypoint_latitudes and waypoint_longitudes length mismatch'
        if names and len(names) != len(latitudes):
            return None, None, None, 'waypoint_names length mismatch'
        if not names:
            names = [f'wp_{index + 1}' for index in range(len(latitudes))]
        return latitudes, longitudes, names, None

    def _waypoint_index_from_payload(self, payload, names, count):
        if 'index' in payload:
            index = int(payload.get('index'))
        elif 'number' in payload:
            index = int(payload.get('number')) - 1
        else:
            name = str(payload.get('name', '')).strip()
            if not name:
                return None, 'missing waypoint index/name'
            try:
                index = names.index(name)
            except ValueError:
                return None, f'waypoint not found: {name}'
        if index < 0 or index >= count:
            return None, f'waypoint index out of range: {index}'
        return index, None

    def _nudge_waypoint(self, payload):
        latitudes, longitudes, names, err = self._load_waypoint_lists()
        if err:
            return False, err, {}
        if not latitudes:
            return False, 'no waypoints configured', {}
        index, err = self._waypoint_index_from_payload(payload, names, len(latitudes))
        if err:
            return False, err, {}

        dx = float(payload.get('dx_m', payload.get('east_m', 0.0)))
        dy = float(payload.get('dy_m', payload.get('north_m', 0.0)))
        origin_lat = latitudes[0]
        origin_lon = longitudes[0]
        x, y = self._latlon_to_enu(latitudes[index], longitudes[index], origin_lat, origin_lon)
        new_lat, new_lon = self._enu_to_latlon(x + dx, y + dy, origin_lat, origin_lon)
        latitudes[index] = new_lat
        longitudes[index] = new_lon

        ok, msg = self._write_waypoint_yaml(latitudes, longitudes, names)
        if not ok:
            return False, msg, {}
        synced, sync_msg = self._sync_waypoints_to_follower()
        detail = {
            'index': index,
            'name': names[index],
            'latitude': new_lat,
            'longitude': new_lon,
            'dx_m': dx,
            'dy_m': dy,
            'synced': synced,
            'sync_message': sync_msg,
        }
        return True, (
            f'nudged {names[index]} by east={dx:.2f}m north={dy:.2f}m'
            + ('' if synced else f' ({sync_msg})')
        ), detail

    def _delete_waypoint(self, payload):
        latitudes, longitudes, names, err = self._load_waypoint_lists()
        if err:
            return False, err, {}
        if not latitudes:
            return False, 'no waypoints configured', {}
        index, err = self._waypoint_index_from_payload(payload, names, len(latitudes))
        if err:
            return False, err, {}
        removed = {
            'index': index,
            'name': names[index],
            'latitude': latitudes[index],
            'longitude': longitudes[index],
        }
        del latitudes[index]
        del longitudes[index]
        del names[index]
        ok, msg = self._write_waypoint_yaml(latitudes, longitudes, names)
        if not ok:
            return False, msg, {}
        synced, sync_msg = self._sync_waypoints_to_follower()
        removed['synced'] = synced
        removed['sync_message'] = sync_msg
        return True, (
            f'deleted {removed["name"]}'
            + ('' if synced else f' ({sync_msg})')
        ), removed

    def handle_command(self, payload):
        if isinstance(payload, str):
            payload = {'cmd': payload}
        payload = payload or {}
        cmd = str(payload.get('cmd') or payload.get('action') or '').strip().lower()

        self.get_logger().info(f'HTTP command: {payload}')

        if cmd in ('ping', 'health'):
            return {'ok': True, 'message': 'pong'}

        if cmd in ('check_control_conflicts', 'check_conflicts', 'control_status'):
            result = self._check_control_conflicts()
            result['message'] = 'ok' if result['ok'] else '; '.join(result['conflicts'])
            return result

        if cmd in ('enable_nav', 'nav_enable'):
            enabled = bool(payload.get('value', True))
            ok, msg = self._set_nav_enabled(enabled)
            return {'ok': ok, 'message': msg, 'enabled': enabled}

        if cmd in ('start_follow', 'start'):
            stop_ok, stop_msg = self._set_nav_enabled(False)
            ok, msg = self._set_nav_enabled(True)
            return {
                'ok': ok,
                'message': msg if stop_ok else f'{msg}; pre-stop: {stop_msg}',
                'enabled': True,
            }

        if cmd in ('set_nav_params', 'set_follow_params', 'set_follow_speed'):
            ok, msg, applied = self._set_nav_params(payload)
            return {'ok': ok, 'message': msg, 'params': applied}

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

        if cmd in ('nudge_waypoint', 'move_waypoint', 'adjust_waypoint'):
            ok, msg, detail = self._nudge_waypoint(payload)
            return {'ok': ok, 'message': msg, 'waypoint': detail}

        if cmd in ('delete_waypoint', 'remove_waypoint'):
            ok, msg, detail = self._delete_waypoint(payload)
            return {'ok': ok, 'message': msg, 'waypoint': detail}

        if cmd in ('cmd_vel', 'teleop'):
            linear_x = float(payload.get('linear_x', 0.0))
            angular_z = float(payload.get('angular_z', 0.0))
            conflict = self._check_control_conflicts()
            if not conflict['ok']:
                self.get_logger().warning(
                    'Rejecting manual cmd_vel due to control conflict: %s'
                    % '; '.join(conflict['conflicts']))
                return {
                    'ok': False,
                    'error': 'control_conflict',
                    'message': '; '.join(conflict['conflicts']),
                    'linear_x': linear_x,
                    'angular_z': angular_z,
                    'conflicts': conflict['conflicts'],
                    'cmd_vel_publishers': conflict['cmd_vel_publishers'],
                    'nav_enabled': conflict['nav_enabled'],
                    'nav_status': conflict['nav_status'],
                }
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
