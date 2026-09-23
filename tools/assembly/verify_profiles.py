#!/usr/bin/env python3
"""Independent DXF reader and numeric checks; never starts simulation.

Requires ezdxf1.4.3 and SciPy. Does not import the generator. The library parses
DXF; SciPy computes polygon halfplanes independently from exported vertices.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import ezdxf
import numpy as np
from scipy.spatial import ConvexHull
import yaml


def read(path):
    doc = ezdxf.readfile(path)
    assert doc.header['$INSUNITS'] == 4, f'{path}: expected millimetres'
    audit = doc.audit()
    assert not audit.errors and not audit.fixes, f'{path}: DXF audit errors/repairs'
    entities = list(doc.modelspace())
    assert all(e.dxftype() in ['LWPOLYLINE', 'CIRCLE', 'ARC', 'LINE'] for e in entities)
    for entity in entities:
        if entity.dxftype() == 'LWPOLYLINE':
            assert entity.closed and len(entity) >= 3
        if entity.dxftype() in ['CIRCLE', 'ARC']:
            assert entity.dxf.radius > 0 and abs(entity.dxf.center.z) < 1e-9
    return entities


def shape(entities):
    types = [e.dxftype() for e in entities]
    if types == ['CIRCLE']:
        e = entities[0]
        return {'kind': 'circle', 'radius_mm': e.dxf.radius,
                'center': np.array(e.dxf.center)[:2]}
    if types == ['LWPOLYLINE']:
        points = np.array(list(entities[0].get_points('xy')))
        hull = ConvexHull(points)
        assert len(hull.vertices) == len(points)
        return {'kind': 'polygon', 'points_mm': points, 'halfplanes': hull.equations,
                'area_mm2': hull.volume}
    assert types == ['ARC', 'LINE'], types
    arc, line = entities
    assert np.linalg.norm(np.array(arc.end_point)-np.array(line.dxf.start)) < 1e-7
    assert np.linalg.norm(np.array(arc.start_point)-np.array(line.dxf.end)) < 1e-7
    assert abs(line.dxf.start.x-line.dxf.end.x) < 1e-7
    angle = (arc.dxf.end_angle-arc.dxf.start_angle) % 360
    assert 180 < angle < 360
    return {'kind': 'd_shape', 'radius_mm': arc.dxf.radius,
            'center': np.array(arc.dxf.center)[:2],
            'clip_x_mm': line.dxf.start.x-arc.dxf.center.x}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists(), 'Evidence output must be new'
    spec = yaml.safe_load((args.input/'source_spec.yaml').read_text())
    source_report = json.loads((args.input/'report.json').read_text())
    for name, expected in source_report['artifacts'].items():
        assert hashlib.sha256((args.input/name).read_bytes()).hexdigest() == expected['sha256']
    checks = []
    for slot in spec['slots']:
        part = shape(read(args.input/f"part_{slot['id']}.dxf"))
        for clearance in spec['clearance']['levels']:
            mm = clearance*1000
            hole = shape(read(args.input/f"hole_{slot['id']}_c{mm:g}mm.dxf"))
            assert part['kind'] == hole['kind']
            if part['kind'] == 'polygon':
                equations = hole['halfplanes']
                margins = -(part['points_mm'] @ equations[:, :2].T + equations[:, 2]).max(axis=0)
                measured = float(margins.min())
                declared_area = source_report['profiles'][slot['id']]['area_m2']*1e6
                assert abs(part['area_mm2']-declared_area) < 1e-6
            else:
                assert np.linalg.norm(part['center']) < 1e-9
                assert np.linalg.norm(hole['center']) < 1e-9
                measured = hole['radius_mm']-part['radius_mm']
                if part['kind'] == 'd_shape':
                    measured = min(measured, hole['clip_x_mm']-part['clip_x_mm'])
                    assert abs(part['clip_x_mm']-10.) < 1e-9
                nominal_radius = slot['profile'].get('radius', slot['profile'].get('diameter', 0)/2)*1000
                assert abs(part['radius_mm']-nominal_radius) < 1e-9
            assert abs(measured-mm) < 1e-7, (slot['id'], measured, mm)
            checks.append({'slot_id': slot['id'], 'nominal_clearance_mm': mm,
                           'readback_clearance_mm': measured})
    for clearance in spec['clearance']['levels']:
        board_entities = read(args.input/f'board_c{clearance*1000:g}mm.dxf')
        assert len(board_entities) == 8, 'Outer contour plus six holes, D uses two entities'
        board = np.array(list(board_entities[0].get_points('xy')))
        assert np.allclose(board.max(axis=0)-board.min(axis=0), [320., 240.], atol=1e-9)
        i = 1
        for slot in spec['slots']:
            count = 2 if slot['profile']['type'] == 'clipped_circle' else 1
            placed = shape(board_entities[i:i+count])
            centered = shape(read(args.input/f"hole_{slot['id']}_c{clearance*1000:g}mm.dxf"))
            shift = np.array(slot['center_in_board'][:2])*1000
            if placed['kind'] == 'polygon':
                assert np.allclose(placed['points_mm']-centered['points_mm'], shift, atol=1e-7)
            else:
                assert np.allclose(placed['center']-centered['center'], shift, atol=1e-7)
            i += count
    result = {'status': 'PASS', 'reader': f'ezdxf {ezdxf.__version__}',
              'independent_polygon_solver': 'scipy.spatial.ConvexHull',
              'dxf_files_read': len(list(args.input.glob('*.dxf'))),
              'matched_clearances_checked': len(checks), 'checks': checks,
              'audit_errors_or_repairs': 0,
              'scope': 'DXF format, mm units, closed contours, dimensions and straight-wall clearance',
              'excluded': ['3D CAD', 'chamfers', 'stem', 'gripper sweep', 'reachability', 'physical insertion'],
              'source_report_sha256': hashlib.sha256((args.input/'report.json').read_bytes()).hexdigest()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'checks'}, indent=2))


if __name__ == '__main__':
    main()
