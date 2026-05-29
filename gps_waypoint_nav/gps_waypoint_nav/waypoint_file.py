import os

import yaml
from ament_index_python.packages import get_package_share_directory


NODE_NAME = 'gps_waypoint_follower'


def _parent_dirs(path):
    current = os.path.abspath(path)
    while True:
        yield current
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent


def _source_waypoint_candidates():
    for base_dir in _parent_dirs(os.getcwd()):
        yield os.path.join(
            base_dir, 'src', 'gps', 'gps_waypoint_nav', 'config', 'waypoints.yaml')
        yield os.path.join(
            base_dir, 'src', 'navigation', 'gps_waypoint_nav', 'config', 'waypoints.yaml')
        yield os.path.join(base_dir, 'gps_waypoint_nav', 'config', 'waypoints.yaml')
        yield os.path.join(base_dir, 'config', 'waypoints.yaml')


def default_waypoint_file():
    for source_path in _source_waypoint_candidates():
        if os.path.exists(source_path):
            return source_path
    return os.path.join(get_package_share_directory('gps_waypoint_nav'), 'config', 'waypoints.yaml')


def load_waypoint_config(path):
    if not os.path.exists(path):
        return {
            NODE_NAME: {
                'ros__parameters': {
                    'enabled': False,
                    'gnss_topic': 'handsfree/rtk/gnss',
                    'cog_topic': 'handsfree/rtk/cog',
                    'heading_topic': 'handsfree/rtk/heading',
                    'cmd_vel_topic': '/cmd_vel',
                    'waypoint_latitudes': [],
                    'waypoint_longitudes': [],
                    'waypoint_names': [],
                }
            }
        }
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    data.setdefault(NODE_NAME, {})
    data[NODE_NAME].setdefault('ros__parameters', {})
    params = data[NODE_NAME]['ros__parameters']
    params.setdefault('heading_topic', 'handsfree/rtk/heading')
    params.setdefault('waypoint_latitudes', [])
    params.setdefault('waypoint_longitudes', [])
    params.setdefault('waypoint_names', [])
    return data


def append_waypoint(path, latitude, longitude, name=None):
    data = load_waypoint_config(path)
    params = data[NODE_NAME]['ros__parameters']
    latitudes = list(params.get('waypoint_latitudes') or [])
    longitudes = list(params.get('waypoint_longitudes') or [])
    names = list(params.get('waypoint_names') or [])

    next_index = len(latitudes) + 1
    waypoint_name = name or 'wp_%d' % next_index

    latitudes.append(float(latitude))
    longitudes.append(float(longitude))
    names.append(str(waypoint_name))

    params['waypoint_latitudes'] = latitudes
    params['waypoint_longitudes'] = longitudes
    params['waypoint_names'] = names

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)

    return waypoint_name, len(latitudes)


def clear_waypoints(path):
    data = load_waypoint_config(path)
    params = data[NODE_NAME]['ros__parameters']
    previous_count = len(params.get('waypoint_latitudes') or [])

    params['waypoint_latitudes'] = []
    params['waypoint_longitudes'] = []
    params['waypoint_names'] = []

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)

    return previous_count
