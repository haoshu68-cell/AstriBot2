#!/usr/bin/env python3
"""Offline corner A/B metrics and static fixture precheck. Never starts ROS."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import median


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vertices(points):
    result = []
    for value in points:
        point = tuple(map(float, value[:2]))
        if not all(map(math.isfinite, point)):
            raise ValueError('nonfinite path')
        if result and math.dist(result[-1], point) < 1e-8:
            continue
        while len(result) >= 2:
            a, b = result[-2:]
            u, v = (b[0]-a[0], b[1]-a[1]), (point[0]-b[0], point[1]-b[1])
            if u[0]*v[0]+u[1]*v[1] <= 0 or abs(u[0]*v[1]-u[1]*v[0]) > 1e-8*math.hypot(*u)*math.hypot(*v):
                break
            result.pop()
        result.append(point)
    return result


def ambiguous_route(points):
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    legs = list(zip(points, points[1:]))
    for i, (a, b) in enumerate(legs):
        for c, d in legs[i+2:]:
            box = all(max(min(a[k], b[k]), min(c[k], d[k])) <=
                      min(max(a[k], b[k]), max(c[k], d[k]))+1e-8 for k in (0, 1))
            if box and cross(a, b, c)*cross(a, b, d) <= 1e-12 and cross(c, d, a)*cross(c, d, b) <= 1e-12:
                return True
    return False


def samples(file, frame):
    result, previous, last_sample = [], None, -math.inf
    with Path(file).open() as stream:
        raw_rows = list(csv.DictReader(stream))
    for row in raw_rows:
        item = {key: float(row[key]) for key in ('t', 'x', 'y', 'yaw')}
        if not all(map(math.isfinite, item.values())) or item['t'] <= 0:
            raise ValueError('invalid source pose/time')
        if row['frame'] != frame:
            raise ValueError('pose/path frame mismatch; transform evidence before comparison')
        if previous and item['t'] < previous['t']:
            raise ValueError('source clock reversed; split execution/clock epochs')
        if previous and item['t'] == previous['t']:
            if item != previous:
                raise ValueError('same source stamp contains conflicting poses')
            continue
        previous = item.copy()
        if item['t']-last_sample < .5-1e-8:
            continue
        last_sample = item['t']
        item['phase'] = row.get('phase', 'UNKNOWN')
        item['leg_index'] = int(row['leg_index']) if row.get('leg_index', '') else None
        result.append(item)
    return result


def spatial_bins(rows, points):
    require_leg = ambiguous_route(points)
    bins, skipped = {}, 0
    for row in rows:
        explicit = row['leg_index']
        if require_leg and explicit is None:
            raise ValueError('self-intersection/repeated route needs recorded ordered leg_index')
        if explicit is not None and not 0 <= explicit < len(points)-1:
            raise ValueError('invalid ordered leg_index')
        projections = []
        for index, (a, b) in enumerate(zip(points, points[1:])):
            if explicit is not None and index != explicit:
                continue
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            along = ((row['x']-a[0])*dx+(row['y']-a[1])*dy)/length
            normal = (dx*(row['y']-a[1])-dy*(row['x']-a[0]))/length
            distance = math.hypot(normal, max(-along, along-length, 0))
            projections.append((distance, index, along, normal, length, math.atan2(dy, dx)))
        if not projections:
            skipped += 1
            continue
        projections.sort()
        distance, index, along, normal, length, heading = projections[0]
        if (len(projections)>1 and abs(distance-projections[1][0])<1e-8) or not .15 <= along <= length-.15:
            skipped += 1
            continue
        key = (index, math.floor(along/.1+1e-8))
        error = abs(math.degrees(math.remainder(row['yaw']-heading, 2*math.pi)))
        bins.setdefault(key, []).append((abs(normal), error))
    return {key: (median(v[0] for v in values), median(v[1] for v in values))
            for key, values in bins.items()}, skipped


def distribution(values):
    if not values:
        return {'n': 0, 'median': None, 'p95': None, 'sampled_max': None}
    a = sorted(values)
    index = .95*(len(a)-1)
    lo = math.floor(index)
    p95 = a[lo]+(a[min(lo+1, len(a)-1)]-a[lo])*(index-lo)
    return {'n': len(a), 'median': median(a), 'p95': p95, 'sampled_max': max(a)}


def compare(reference, candidate):
    packs = []
    common_frame = None
    for directory in (reference, candidate):
        route_file, motion_file = directory/'path.json', directory/'motion.csv'
        route = json.loads(route_file.read_text())
        if common_frame is not None and route['frame'] != common_frame:
            raise ValueError('A/B path frames differ')
        common_frame = route['frame']
        points = vertices(route['points'])
        rows = samples(motion_file, route['frame'])
        bins, skipped = spatial_bins(rows, points)
        packs.append((points, bins, {'source_directory': str(directory), 'sampled_pose_count': len(rows),
                     'excluded_endpoint_or_ambiguous_samples': skipped,
                     'phase_counts': {p: sum(r['phase']==p for r in rows) for p in sorted({r['phase'] for r in rows})},
                     'sha256': {'path.json': digest(route_file), 'motion.csv': digest(motion_file)}}))
    if packs[0][0] != packs[1][0]:
        raise ValueError('A/B directed route geometry differs')
    common = sorted(set(packs[0][1]) & set(packs[1][1]))
    for _, bins, report in packs:
        report['observed_spatial_bins'] = len(bins)
        report['common_bin_fraction'] = len(common)/len(bins) if bins else 0.
        report['lateral_m'] = distribution([bins[k][0] for k in common])
        report['heading_deg'] = distribution([bins[k][1] for k in common])
    return {'status': 'STATISTICS_ONLY' if common else 'UNAVAILABLE_NO_COMMON_SPACE',
            'scope': 'Offline sampled spatial comparison, not controller/robot acceptance. No time score.',
            'method': 'At most 2 Hz source pose; same directed leg and 0.1 m bins; 0.15 m endpoint exclusion; no FOLLOW-phase filter. Equal weight per shared bin median. Self intersections require recorded ordered leg_index.',
            'common_bins': len(common), 'reference': packs[0][2], 'candidate': packs[1][2]}


def fixture(map_file, scenario):
    import numpy as np
    import yaml
    from PIL import Image
    from scipy.ndimage import distance_transform_edt
    m = yaml.safe_load(map_file.read_text())
    if (m.get('mode', 'trinary') != 'trinary' or m['negate'] not in (0, 1) or
            not math.isfinite(m['resolution']) or m['resolution'] <= 0 or
            not 0 < m['free_thresh'] < m['occupied_thresh'] < 1 or
            len(m['origin']) != 3 or not all(map(math.isfinite, m['origin']))):
        raise ValueError('invalid or unsupported static map metadata')
    image_file = map_file.parent/m['image']
    image = np.asarray(Image.open(image_file).convert('L'), dtype=float)/255.
    occupancy = image if m['negate'] else 1.-image
    free = occupancy < m['free_thresh']
    # Padding makes map boundaries nonfree. Distance is between raster centers;
    # subtract a full cell diagonal for the arbitrary query/cell extents.
    clearance = distance_transform_edt(np.pad(free, 1, constant_values=False))[1:-1, 1:-1]*m['resolution']
    height, width = free.shape
    config = yaml.safe_load(scenario.read_text())['hunav_loader']['ros__parameters']
    ox, oy, yaw = m['origin']; co, si = math.cos(yaw), math.sin(yaw)
    records = []
    for name in config['agents']:
        person = config[name]
        points = [(person['init_pose']['x'], person['init_pose']['y'])]
        points += [(config['global_goals'][g]['x'], config['global_goals'][g]['y']) for g in person['goals']]
        if (not math.isfinite(person['radius']) or person['radius'] <= 0 or
                not all(math.isfinite(value) for point in points for value in point)):
            raise ValueError('invalid person radius/route')
        if len(points) == 1:
            points.append(points[0])  # Stationary agents still need a static check.
        elif person.get('cyclic_goals', False) and len(points) > 2:
            points.append(points[1])  # Cycle goals; do not return to the spawn pose.
        minimum = math.inf; count = 0
        for a, b in zip(points, points[1:]):
            n = math.ceil(math.dist(a, b)/(.5*m['resolution']))
            for i in range(n+1):
                x, y = (a[k]+(b[k]-a[k])*i/max(n, 1) for k in (0, 1))
                dx, dy = x-ox, y-oy
                col = math.floor((co*dx+si*dy)/m['resolution'])
                row = height-1-math.floor((-si*dx+co*dy)/m['resolution'])
                gap = float(clearance[row,col])-math.sqrt(2)*m['resolution']-person['radius'] if 0<=row<height and 0<=col<width else -math.inf
                minimum = min(minimum, gap); count += 1
        records.append({'person': name, 'sampled_reference_disk_static_gap_m': minimum if math.isfinite(minimum) else None,
                        'reference_clear': count>0 and minimum>=0, 'samples': count})
    return {'status': 'REFERENCE_PRECHECK_ONLY', 'agents': records,
            'scope': 'Planned center line against static raster only. Actual HuNav motion, robot/person clearance, mesh/height, full envelope and Gazebo/RViz views remain NOT_RUN.',
            'sha256': {'map_metadata': digest(map_file), 'map_image': digest(image_file), 'scenario': digest(scenario)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    commands = parser.add_subparsers(dest='mode', required=True)
    c = commands.add_parser('compare')
    c.add_argument('--reference', type=Path, required=True); c.add_argument('--candidate', type=Path, required=True)
    f = commands.add_parser('fixture')
    f.add_argument('--map', type=Path, required=True); f.add_argument('--scenario', type=Path, required=True)
    args = parser.parse_args()
    try:
        report = compare(args.reference, args.candidate) if args.mode=='compare' else fixture(args.map, args.scenario)
    except (ValueError, KeyError, OSError) as error:
        report = {'status': 'INVALID_EVIDENCE', 'reason': str(error)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(report, indent=2, allow_nan=False))
    return 2 if report['status'].startswith(('INVALID', 'UNAVAILABLE')) else 0


if __name__ == '__main__':
    raise SystemExit(main())
