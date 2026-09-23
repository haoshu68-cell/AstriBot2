#!/usr/bin/env python3
"""Generate/screen original 2D CAD profiles. No robot, ROS or GPU side effects.

This operations tool does not create a physical scene or certify 3D insertion.
Circle and D profiles remain analytic; DXF arcs are not sampled polygons.
"""
import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
from functools import cached_property
import hashlib
import json
import math
from pathlib import Path
import re
import tempfile

import yaml

EPS = 1e-10


@dataclass(frozen=True)
class Profile:
    kind: str
    vertices: tuple = ()
    radius: float = 0.
    clip_x: float = 0.

    def __post_init__(self):
        if self.kind == 'polygon':
            if len(self.vertices) < 3:
                raise ValueError('Polygon needs at least three vertices')
            for i, a in enumerate(self.vertices):
                b, c = self.vertices[(i+1) % len(self.vertices)], self.vertices[(i+2) % len(self.vertices)]
                values = (*a, *b, *c)
                cross = (b[0]-a[0])*(c[1]-b[1]) - (b[1]-a[1])*(c[0]-b[0])
                if not all(math.isfinite(x) for x in values) or cross <= 0:
                    raise ValueError('Expected strictly convex CCW polygon')
            if any(nx*x+ny*y > bound+1e-12 for nx, ny, bound in self.planes
                   for x, y in self.vertices):
                raise ValueError('Polygon is not a simple convex support-line intersection')
        elif self.kind in ['circle', 'd_shape']:
            if not math.isfinite(self.radius) or self.radius <= 0:
                raise ValueError('Positive finite radius required')
            if self.kind == 'd_shape' and not 0 < self.clip_x < self.radius:
                raise ValueError('This candidate D profile requires 0 < clip_x < radius')
        else:
            raise ValueError('Unknown analytic profile')

    @staticmethod
    def polygon(vertices):
        return Profile('polygon', tuple(tuple(float(x) for x in v) for v in vertices))

    @cached_property
    def planes(self):
        if self.kind != 'polygon':
            raise ValueError('Only polygons have finite support-line sets')
        result = []
        for a, b in zip(self.vertices, self.vertices[1:] + self.vertices[:1]):
            dx, dy = b[0]-a[0], b[1]-a[1]
            length = math.hypot(dx, dy)
            nx, ny = dy/length, -dx/length
            result.append((nx, ny, nx*a[0] + ny*a[1]))
        return tuple(result)

    def support(self, nx, ny):
        norm = math.hypot(nx, ny)
        if norm == 0:
            return 0.
        if self.kind == 'polygon':
            return max(nx*x + ny*y for x, y in self.vertices)
        if self.kind == 'circle' or self.radius*nx/norm <= self.clip_x:
            return self.radius * norm
        return self.clip_x*nx + math.sqrt(self.radius**2-self.clip_x**2)*abs(ny)

    def max_radius(self):
        return (max(math.hypot(x, y) for x, y in self.vertices)
                if self.kind == 'polygon' else self.radius)

    def bounds(self):
        return (-self.support(-1., 0.), -self.support(0., -1.),
                self.support(1., 0.), self.support(0., 1.))

    def area(self):
        if self.kind == 'circle':
            return math.pi * self.radius**2
        if self.kind == 'd_shape':
            angle = math.acos(self.clip_x / self.radius)
            return ((math.pi-angle)*self.radius**2
                    + self.clip_x*math.sqrt(self.radius**2-self.clip_x**2))
        return .5 * sum(a[0]*b[1]-a[1]*b[0]
                        for a, b in zip(self.vertices, self.vertices[1:]+self.vertices[:1]))

    def record(self):
        return {'kind': self.kind, 'vertices_m': self.vertices, 'radius_m': self.radius,
                'clip_x_m': self.clip_x, 'bounds_m': self.bounds(), 'area_m2': self.area()}


def offset_profile(profile, clearance):
    if not math.isfinite(clearance) or clearance < 0:
        raise ValueError('Nonnegative finite normal clearance required')
    if profile.kind == 'circle':
        return Profile('circle', radius=profile.radius+clearance)
    if profile.kind == 'd_shape':
        return Profile('d_shape', radius=profile.radius+clearance,
                       clip_x=profile.clip_x+clearance)
    vertices = []
    for i, (nx, ny, bound) in enumerate(profile.planes):
        px, py, previous = profile.planes[i-1]
        determinant = px*ny-py*nx
        if abs(determinant) < 1e-12:
            raise ValueError('Parallel adjacent support lines')
        b0, b1 = previous+clearance, bound+clearance
        vertices.append(((b0*ny-py*b1)/determinant, (px*b1-b0*nx)/determinant))
    return Profile.polygon(vertices)


def clearance_margin(part, hole, yaw):
    co, si = math.cos(yaw), math.sin(yaw)
    def support(nx, ny):
        return part.support(co*nx+si*ny, -si*nx+co*ny)
    if hole.kind == 'polygon':
        return min(bound-support(nx, ny) for nx, ny, bound in hole.planes)
    radial = hole.radius-part.max_radius()
    if hole.kind == 'circle':
        return radial
    return min(radial, hole.clip_x-support(1., 0.))


def screen_pair(part, hole, yaw_step_deg):
    count = round(360/yaw_step_deg)
    if count < 2 or abs(count*yaw_step_deg-360) > EPS:
        raise ValueError('Yaw step must divide 360 degrees')
    best, best_yaw = -math.inf, 0.
    for i in range(count):
        yaw = math.radians(i*yaw_step_deg)
        margin = clearance_margin(part, hole, yaw)
        if margin > best + 1e-12:
            best, best_yaw = margin, yaw
    classification = ('POSITIVE_CLEARANCE_WITNESS' if best > EPS else
                      'TANGENCY_ONLY_WITNESS' if best >= -EPS else
                      'NO_FIT_IN_SAMPLED_CENTERED_POSES')
    return {'classification': classification, 'best_sampled_clearance_m': best,
            'witness_yaw_rad': best_yaw, 'translation_xy_m': [0., 0.],
            'yaw_step_deg': yaw_step_deg, 'business_success': False,
            'proves_no_fit_under_arbitrary_pose': False}


def profile_from_slot(slot):
    p = slot['profile']
    kind = p['type']
    if kind == 'circle':
        return Profile('circle', radius=p['diameter']/2)
    if kind == 'clipped_circle':
        match = re.fullmatch(r'x\s*<=\s*([0-9.eE+-]+)', p['retain_halfplane'])
        if not match:
            raise ValueError('Expected explicit x <= a halfplane')
        return Profile('d_shape', radius=p['radius'], clip_x=float(match[1]))
    if kind in ['rectangle', 'clipped_rectangle']:
        w, h = p['size_xy']
        if w <= 0 or h <= 0:
            raise ValueError('Positive rectangle dimensions required')
        vertices = [(-w/2, -h/2), (w/2, -h/2)]
        if kind == 'clipped_rectangle':
            if p['remove_corner'] != 'positive_x_positive_y':
                raise ValueError('Unsupported key corner')
            cx, cy = p['corner_cut_legs']
            if not 0 < cx < w or not 0 < cy < h:
                raise ValueError('Invalid key cut')
            vertices += [(w/2, h/2-cy), (w/2-cx, h/2)]
        else:
            vertices += [(w/2, h/2)]
        return Profile.polygon(vertices + [(-w/2, h/2)])
    if kind == 'regular_triangle':
        if p['vertex_direction'] != 'positive_y':
            raise ValueError('Unsupported triangle phase')
        n, radius, phase = 3, p['circumradius'], math.pi/2
    elif kind == 'regular_hexagon':
        if p['vertex_direction'] != 'positive_x':
            raise ValueError('Unsupported hexagon phase')
        n, radius, phase = 6, p['across_flats']/2/math.cos(math.pi/6), 0.
    else:
        raise ValueError('Unsupported profile type')
    return Profile.polygon([(radius*math.cos(phase+2*i*math.pi/n),
                             radius*math.sin(phase+2*i*math.pi/n)) for i in range(n)])


def dxf_text(profiles):
    """Minimal ASCII DXF in millimetres; exact circles/arcs, closed polylines."""
    pairs = [(0, 'SECTION'), (2, 'HEADER'), (9, '$ACADVER'), (1, 'AC1015'),
             (9, '$INSUNITS'), (70, 4), (9, '$MEASUREMENT'), (70, 1),
             (0, 'ENDSEC'), (0, 'SECTION'), (2, 'ENTITIES')]
    def entity(name, fields):
        pairs.extend([(0, name), (8, '0'), *fields])
    for profile, tx, ty in profiles:
        tx, ty = tx*1000, ty*1000
        if profile.kind == 'polygon':
            fields = [(90, len(profile.vertices)), (70, 1)]
            for x, y in profile.vertices:
                fields.extend([(10, x*1000+tx), (20, y*1000+ty)])
            entity('LWPOLYLINE', fields)
        elif profile.kind == 'circle':
            entity('CIRCLE', [(10, tx), (20, ty), (30, 0.), (40, profile.radius*1000)])
        else:
            radius, x = profile.radius*1000, profile.clip_x*1000
            y = math.sqrt(radius*radius-x*x)
            angle = math.degrees(math.acos(x/radius))
            entity('ARC', [(10, tx), (20, ty), (30, 0.), (40, radius),
                           (50, angle), (51, 360-angle)])
            entity('LINE', [(10, tx+x), (20, ty-y), (30, 0.),
                            (11, tx+x), (21, ty+y), (31, 0.)])
    pairs.extend([(0, 'ENDSEC'), (0, 'EOF')])
    return ''.join(f'{code}\n{value:.12g}\n' if isinstance(value, float)
                   else f'{code}\n{value}\n' for code, value in pairs)


def svg_element(profile, cx, cy, fill, stroke):
    style = f'fill="{fill}" stroke="{stroke}" stroke-width="0.45"'
    if profile.kind == 'polygon':
        points = ' '.join(f'{cx+x*1000:.8f},{cy-y*1000:.8f}' for x, y in profile.vertices)
        return f'<polygon points="{points}" {style}/>'
    radius = profile.radius*1000
    if profile.kind == 'circle':
        return f'<circle cx="{cx}" cy="{cy}" r="{radius}" {style}/>'
    x, y = profile.clip_x*1000, math.sqrt(profile.radius**2-profile.clip_x**2)*1000
    return (f'<path d="M {cx+x} {cy-y} A {radius} {radius} 0 1 0 {cx+x} {cy+y} Z" '
            f'{style}/>')


def generate(spec, output, step):
    data = yaml.safe_load(spec.read_text())
    if (data['status'] != 'DESIGN_ONLY' or data['units']['length'] != 'm'
            or data['geometry_contract_revision'] != 2 or data['execution_authorized'] is not False):
        raise ValueError('Expected explicit metre-based non-executable candidate revision2')
    if output.exists():
        raise ValueError('Output must be a new immutable directory')
    slots, board = data['slots'], data['board']
    ids = [s['id'] for s in slots]
    if len(set(ids)) != len(ids) or not all(re.fullmatch('[a-z][a-z0-9_]*', s) for s in ids):
        raise ValueError('Invalid/duplicate slot identifiers')
    w, h, thickness = board['dimensions']
    if abs(thickness-board['blind_depth']-board['floor_thickness']) > EPS:
        raise ValueError('Blind depth and floor thickness disagree')
    profiles = {s['id']: profile_from_slot(s) for s in slots}
    board_profile = Profile.polygon([(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)])
    report = {'schema': 'astribot.assembly.profile_screening/1',
              'created_at': datetime.now(timezone.utc).isoformat(),
              'evidence_layer': 'offline_analytic_cross_section', 'execution_authorized': False,
              'contact_validated': False, 'complete_s0_or_scene_acceptance': False,
              'scope': 'centered 2D straight-wall sections; sampled yaw; no chamfer, stem or gripper',
              'source_spec_sha256': hashlib.sha256(spec.read_bytes()).hexdigest(),
              'd_hole_convention': data['clearance']['d_shape_method'],
              'profiles': {name: p.record() for name, p in profiles.items()},
              'nominal_matches': [], 'cross_fit': [], 'artifacts': {}}
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output.parent, prefix=output.name+'.partial-') as temporary:
        root = Path(temporary)
        for name, p in profiles.items():
            (root/f'part_{name}.dxf').write_text(dxf_text([(p, 0., 0.)]))
        for c in data['clearance']['levels']:
            if c <= 0:
                raise ValueError('Positive candidate clearance required')
            suffix = f'{c*1000:g}mm'
            placed = [(board_profile, 0., 0.)]
            svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1520" height="1280" viewBox="0 0 380 320">',
                   '<rect width="380" height="320" fill="#f4f7fa"/>',
                   f'<text x="30" y="18" font-family="sans-serif" font-size="9" fill="#1c344a">Assembly profiles | normal clearance {c*1000:g} mm</text>',
                   '<rect x="30" y="30" width="320" height="240" rx="2" fill="#e0e7ed" stroke="#60768a"/>']
            for slot in slots:
                name, p = slot['id'], profiles[slot['id']]
                hole = offset_profile(p, c)
                margin = clearance_margin(p, hole, 0.)
                if abs(margin-c) > EPS:
                    raise ValueError('Nominal matching clearance disagrees with specification')
                x0, y0, x1, y1 = hole.bounds()
                cx, cy, cz = slot['center_in_board']
                if abs(cz) > EPS or abs(slot['yaw_in_board']) > EPS:
                    raise ValueError('This first board layout assumes z=0, yaw=0')
                mx, my = board['module_footprint']
                module_margin = min(x0+mx/2, y0+my/2, mx/2-x1, my/2-y1)
                board_margin = min(cx+x0+w/2, cy+y0+h/2, w/2-cx-x1, h/2-cy-y1)
                if min(module_margin, board_margin) <= 0:
                    raise ValueError('Hole escapes candidate board or module')
                report['nominal_matches'].append({'slot_id': name, 'clearance_m': c,
                    'measured_analytic_margin_m': margin, 'module_edge_margin_m': module_margin,
                    'board_edge_margin_m': board_margin, 'hole': hole.record()})
                filename = f'hole_{name}_c{suffix}.dxf'
                (root/filename).write_text(dxf_text([(hole, 0., 0.)]))
                placed.append((hole, cx, cy))
                sx, sy = 190+cx*1000, 150-cy*1000
                svg.extend([f'<rect x="{sx-40}" y="{sy-40}" width="80" height="80" fill="none" stroke="#9babba" stroke-dasharray="2 2"/>',
                            svg_element(hole, sx, sy, '#ffffff', '#487ba1'),
                            svg_element(p, sx, sy, '#f0bb64', '#795320'),
                            f'<text x="{sx}" y="{sy+47}" text-anchor="middle" font-family="sans-serif" font-size="5" fill="#233a4c">{name}</text>'])
                for source, part in profiles.items():
                    fit = screen_pair(part, hole, step)
                    fit.update({'part_id': source, 'slot_id': name, 'clearance_level_m': c,
                                'assigned_match': source == name})
                    report['cross_fit'].append(fit)
            (root/f'board_c{suffix}.dxf').write_text(dxf_text(placed))
            svg += ['<text x="30" y="291" font-family="sans-serif" font-size="5.5" fill="#324b60">Orange: part | White: hole | Dashed: 80 mm module | Board: 320 x 240 mm</text>',
                    '<text x="30" y="304" font-family="sans-serif" font-size="5.5" fill="#6e7880">2D candidate only. No chamfers, stem, gripper sweep, contact or task success.</text>', '</svg>']
            (root/f'board_c{suffix}.svg').write_text('\n'.join(svg)+'\n')
        (root/'source_spec.yaml').write_bytes(spec.read_bytes())
        (root/'generator.py').write_bytes(Path(__file__).read_bytes())
        for p in sorted(root.iterdir()):
            report['artifacts'][p.name] = {'sha256': hashlib.sha256(p.read_bytes()).hexdigest(),
                                         'bytes': p.stat().st_size}
        wrong = [r for r in report['cross_fit'] if not r['assigned_match']
                 and r['classification'] == 'POSITIVE_CLEARANCE_WITNESS']
        report['summary'] = {'nominal_matches_passed': len(report['nominal_matches']),
                             'screened_combinations': len(report['cross_fit']),
                             'wrong_slot_positive_witnesses': len(wrong),
                             'geometric_fit_implies_business_success': False}
        (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        root.rename(output)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--yaw-step-deg', type=float, default=.5)
    args = parser.parse_args()
    if not math.isfinite(args.yaw_step_deg) or not .1 <= args.yaw_step_deg <= 180:
        parser.error('Yaw step must be finite within [.1, 180] degrees')
    report = generate(args.spec, args.output, args.yaw_step_deg)
    print(json.dumps(report['summary'], indent=2))


if __name__ == '__main__':
    main()
