#!/usr/bin/env python3
import argparse
import math
import os
import xml.sax.saxutils as xml_escape

import yaml


def repo_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def load_waypoints(path):
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    params = data.get('gps_waypoint_follower', {}).get('ros__parameters', {})
    latitudes = list(params.get('waypoint_latitudes') or [])
    longitudes = list(params.get('waypoint_longitudes') or [])
    names = list(params.get('waypoint_names') or [])

    if len(latitudes) != len(longitudes):
        raise ValueError('waypoint_latitudes and waypoint_longitudes length mismatch')
    if len(names) < len(latitudes):
        names.extend('wp_%d' % (i + 1) for i in range(len(names), len(latitudes)))

    return [(str(names[i]), float(latitudes[i]), float(longitudes[i]))
            for i in range(len(latitudes))]


def to_enu(points):
    if not points:
        return []
    radius_m = 6378137.0
    lat0 = math.radians(points[0][1])
    lon0 = math.radians(points[0][2])
    result = []
    for name, lat, lon in points:
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        x = radius_m * (lon_rad - lon0) * math.cos((lat_rad + lat0) / 2.0)
        y = radius_m * (lat_rad - lat0)
        result.append((name, x, y, lat, lon))
    return result


def segment_distances(points):
    return [
        (a[0], b[0], math.hypot(b[1] - a[1], b[2] - a[2]))
        for a, b in zip(points, points[1:])
    ]


def nice_grid_step(span):
    if span <= 8:
        return 1.0
    if span <= 20:
        return 2.0
    if span <= 50:
        return 5.0
    return 10.0


def svg_text(x, y, text, cls):
    return '<text class="%s" x="%.1f" y="%.1f">%s</text>' % (
        cls, x, y, xml_escape.escape(str(text)))


def make_svg(points, distances):
    width = 900
    height = 680
    margin = 90
    if not points:
        return '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d"></svg>\n' % (
            width, height)

    xs = [p[1] for p in points]
    ys = [p[2] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y, 55.0)
    center_x = (min_x + max_x) / 2.0
    center_y = (min_y + max_y) / 2.0

    def screen(x, y):
        return width / 2.0 + (x - center_x) * scale, height / 2.0 - (y - center_y) * scale

    plot_points = [(name, *screen(x, y), x, y, lat, lon)
                   for name, x, y, lat, lon in points]
    polyline = ' '.join('%.1f,%.1f' % (p[1], p[2]) for p in plot_points)

    grid_step = nice_grid_step(max(span_x, span_y))
    start_x = math.floor((min_x - 2) / grid_step) * grid_step
    end_x = math.ceil((max_x + 2) / grid_step) * grid_step
    start_y = math.floor((min_y - 2) / grid_step) * grid_step
    end_y = math.ceil((max_y + 2) / grid_step) * grid_step

    grid_lines = []
    x = start_x
    while x <= end_x + 1e-9:
        sx1, sy1 = screen(x, start_y)
        sx2, sy2 = screen(x, end_y)
        cls = 'axis' if abs(x) < 1e-9 else 'grid'
        grid_lines.append(
            '<line class="%s" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
            % (cls, sx1, sy1, sx2, sy2))
        x += grid_step

    y = start_y
    while y <= end_y + 1e-9:
        sx1, sy1 = screen(start_x, y)
        sx2, sy2 = screen(end_x, y)
        cls = 'axis' if abs(y) < 1e-9 else 'grid'
        grid_lines.append(
            '<line class="%s" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
            % (cls, sx1, sy1, sx2, sy2))
        y += grid_step

    labels = []
    for i, (name, sx, sy, x, y, lat, lon) in enumerate(plot_points):
        labels.append('<circle class="pt" cx="%.1f" cy="%.1f" r="8"/>' % (sx, sy))
        labels.append(svg_text(sx + 11, sy - 11, '%s (%.1fm, %.1fm)' % (name, x, y), 'label'))
        labels.append(svg_text(sx + 11, sy + 9, '%.8f, %.8f' % (lat, lon), 'coord'))
        if i > 0:
            px, py = plot_points[i - 1][1], plot_points[i - 1][2]
            labels.append(svg_text((px + sx) / 2.0 + 8, (py + sy) / 2.0 - 8,
                                   '%.2fm' % distances[i - 1][2], 'dist'))

    total = sum(d[2] for d in distances)
    summary = [
        svg_text(28, 38, 'GPS Waypoints Preview', 'title'),
        svg_text(28, 64, 'origin: %s, ENU plane, grid: %.1fm' % (points[0][0], grid_step), 'subtitle'),
        svg_text(28, 90, 'points: %d, path length: %.2fm' % (len(points), total), 'subtitle'),
    ]

    return '''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <style>
    .bg {{ fill: #f8fafc; }}
    .grid {{ stroke: #dbe3ef; stroke-width: 1; }}
    .axis {{ stroke: #94a3b8; stroke-width: 2; }}
    .path {{ fill: none; stroke: #2563eb; stroke-width: 4; stroke-linecap: round; stroke-linejoin: round; }}
    .pt {{ fill: #dc2626; stroke: #ffffff; stroke-width: 3; }}
    .title {{ font: 22px sans-serif; fill: #0f172a; font-weight: 700; }}
    .subtitle {{ font: 14px sans-serif; fill: #475569; }}
    .label {{ font: 14px sans-serif; fill: #0f172a; font-weight: 700; }}
    .coord {{ font: 11px monospace; fill: #64748b; }}
    .dist {{ font: 13px sans-serif; fill: #1d4ed8; font-weight: 700; }}
  </style>
  <rect class="bg" x="0" y="0" width="{width}" height="{height}"/>
  {summary}
  <g>
    {grid}
    <polyline class="path" points="{polyline}"/>
    {labels}
  </g>
</svg>
'''.format(width=width, height=height, summary='\n  '.join(summary),
           grid='\n    '.join(grid_lines), polyline=polyline,
           labels='\n    '.join(labels))


def main():
    root = repo_root()
    parser = argparse.ArgumentParser(description='Generate an SVG preview from GPS waypoint YAML.')
    parser.add_argument('input', nargs='?',
                        default=os.path.join(root, 'gps_waypoint_nav', 'config', 'waypoints.yaml'))
    parser.add_argument('-o', '--output',
                        default=os.path.join(root, 'gps_waypoints_preview.svg'))
    args = parser.parse_args()

    points = to_enu(load_waypoints(args.input))
    distances = segment_distances(points)
    with open(args.output, 'w', encoding='utf-8') as stream:
        stream.write(make_svg(points, distances))

    print('Wrote %s' % args.output)
    for name, x, y, _lat, _lon in points:
        print('%s: x=%.2fm east, y=%.2fm north' % (name, x, y))
    for a, b, distance in distances:
        print('%s -> %s: %.2fm' % (a, b, distance))
    print('total: %.2fm' % sum(d[2] for d in distances))


if __name__ == '__main__':
    main()
