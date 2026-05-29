import math


EARTH_RADIUS_M = 6378137.0


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def latlon_to_enu(lat, lon, origin_lat, origin_lon):
    lat_rad = math.radians(lat)
    origin_lat_rad = math.radians(origin_lat)
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)
    x = EARTH_RADIUS_M * d_lon * math.cos(origin_lat_rad)
    y = EARTH_RADIUS_M * d_lat
    return x, y


def enu_to_latlon(x, y, origin_lat, origin_lon):
    origin_lat_rad = math.radians(origin_lat)
    lat_rad = origin_lat_rad + y / EARTH_RADIUS_M
    lon_rad = math.radians(origin_lon) + x / (EARTH_RADIUS_M * math.cos(origin_lat_rad))
    return math.degrees(lat_rad), math.degrees(lon_rad)


def enu_distance_and_bearing(current_xy, target_xy):
    dx = target_xy[0] - current_xy[0]
    dy = target_xy[1] - current_xy[1]
    distance = math.hypot(dx, dy)
    bearing = math.atan2(dx, dy)
    return distance, bearing
