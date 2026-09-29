"""Stateful, same-input native differential against a frozen test-only oracle.

Run with POLICY_HEALTH_PROBE=/absolute/native/probe python3 -m pytest -q this_file.
There is deliberately no production Python package import or runtime fallback.
"""
from dataclasses import asdict, is_dataclass
from enum import Enum
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parent
REFERENCE = ROOT / 'reference/policy_health'
PACKAGE = '_frozen_policy_health_20260921'
spec = importlib.util.spec_from_file_location(
    PACKAGE, REFERENCE / '__init__.py', submodule_search_locations=[str(REFERENCE)])
oracle_package = importlib.util.module_from_spec(spec)
sys.modules[PACKAGE] = oracle_package
spec.loader.exec_module(oracle_package)
previous_native = os.environ.pop('ASTRIBOT_NAV_NATIVE_KERNELS', None)
try:
    contracts = __import__(PACKAGE + '.contracts', fromlist=['contracts'])
    oracle = __import__(PACKAGE + '.sensor_health', fromlist=['sensor_health'])
finally:
    if previous_native is not None:
        os.environ['ASTRIBOT_NAV_NATIVE_KERNELS'] = previous_native
assert oracle._native is None


def value(data):
    if is_dataclass(data):
        return value(asdict(data))
    if isinstance(data, Enum):
        return data.value
    if isinstance(data, (list, tuple)):
        return [value(item) for item in data]
    if isinstance(data, dict):
        return {key: value(item) for key, item in data.items()}
    return data


def number(item):
    return {'nan': math.nan, 'inf': math.inf, '-inf': -math.inf}.get(item, item) if isinstance(item, str) else item


def stamp(ns, epoch=0, clock='ros'):
    return dict(ns=ns, clock=clock, epoch=epoch)


def cone(angle=0., half=math.pi):
    return dict(direction=dict(x=math.cos(angle), y=math.sin(angle), z=0.), half_angle_rad=half)


def record(ns, now=None, sensor='scan', epoch=0, calibration=0, cones=None, depth=True, frame='base_link'):
    return dict(op='record', sensor=sensor, capture=stamp(ns, epoch),
                now=stamp(ns if now is None else now, epoch), frame=frame,
                coverage=[cone()] if cones is None else cones, depth=depth, calibration_epoch=calibration)


def query(ns, epoch=0, clock='ros'):
    return dict(op='health', now=stamp(ns, epoch, clock))


def calibration(**changes):
    result = dict(camera_id='front', optical_frame='front_optical', calibration_epoch=1,
                  image_width_px=640, image_height_px=480,
                  intrinsic_matrix=[500., 0., 320., 0., 500., 240., 0., 0., 1.],
                  distortion_model='plumb_bob', distortion_coefficients=[0., 0., 0., 0., 0.],
                  depth_unit_m=.001)
    result.update(changes)
    return result


def reference(case):
    registry = oracle.SensorHealthRegistry(case.get('timeout', 1e-7), case.get('required', ['scan']))
    cameras = oracle.CameraCalibrationRegistry()
    result = []
    for op in case['ops']:
        try:
            name = op['op']
            if name == 'record':
                cones = tuple(contracts.BearingCone(contracts.Vec3(**c['direction']), number(c['half_angle_rad'])) for c in op['coverage'])
                item = registry.record(op['sensor'], contracts.Stamp(**op['capture']),
                    contracts.Stamp(**op['now']), op['frame'], cones, op['depth'], op['calibration_epoch'])
            elif name in ('health', 'required_valid'):
                item = getattr(registry, name)(contracts.Stamp(**op['now']))
            elif name == 'allows':
                item = registry.allows(contracts.Stamp(**op['now']), op['directions'])
            elif name == 'allows_motion':
                item = registry.allows_motion(contracts.Stamp(**op['now']), *op['velocity'])
            elif name == 'scan':
                item = oracle.scan_coverage([number(n) for n in op['ranges']], *op['parameters'])
            elif name == 'directions':
                item = oracle.movement_directions(*op['velocity'])
            elif name == 'coverage_motion':
                cones = tuple(contracts.BearingCone(contracts.Vec3(**c['direction']), c['half_angle_rad']) for c in op['coverage'])
                item = oracle.coverage_allows_motion(cones, *op['velocity'])
            elif name == 'calibrate':
                c = dict(op['calibration'])
                for field in ('intrinsic_matrix', 'distortion_coefficients'):
                    c[field] = tuple(number(n) for n in c[field])
                c['depth_unit_m'] = number(c['depth_unit_m'])
                item = cameras.register(contracts.CameraCalibration(**c))
            elif name == 'calibration':
                item = cameras.calibration(op['camera_id'], op['epoch'])
            else:
                raise AssertionError(name)
            result.append(dict(value=value(item)))
        except (ValueError, KeyError) as error:
            result.append(dict(error=str(error)))
    return result


def compare_values(expected, actual, path=''):
    if isinstance(expected, dict):
        assert set(expected) == set(actual), path
        for key in expected:
            compare_values(expected[key], actual[key], path + '/' + key)
    elif isinstance(expected, list):
        assert len(expected) == len(actual), path
        for index, (left, right) in enumerate(zip(expected, actual)):
            compare_values(left, right, f'{path}/{index}')
    elif isinstance(expected, float):
        assert actual == pytest.approx(expected, abs=2e-14, rel=2e-14), path
    else:
        assert type(actual) is type(expected), f'{path}: scalar type mismatch'
        assert actual == expected, f'{path}: {actual!r} != {expected!r}'


def run(case):
    binary = os.environ.get('POLICY_HEALTH_PROBE')
    assert binary and Path(binary).is_file(), 'native policy health probe has not been implemented/built'
    completed = subprocess.run([binary], input=json.dumps(case) + '\n', capture_output=True,
                               text=True, timeout=15, check=True)
    actual = json.loads(completed.stdout)
    expected = reference(case)
    compare_values(expected, actual)
    return actual


def test_frozen_oracle_hashes():
    for name, item in json.loads((REFERENCE / 'manifest.json').read_text()).items():
        assert hashlib.sha256((REFERENCE / name).read_bytes()).hexdigest() == item['sha256']


def test_acquisition_expiry_equality_and_ordering():
    result = run(dict(ops=[record(100, 140), query(199), query(200), query(201), query(99),
                          record(100, 200), record(99, 150), record(101, 201), query(201)]))
    assert result[0] == {'value': True}
    assert [result[i]['value'][0]['health'] for i in (1, 2, 3, 4)] == ['VALID', 'VALID', 'STALE', 'STALE']
    assert result[1]['value'][0]['stamp']['ns'] == 100
    assert result[1]['value'][0]['valid_until']['ns'] == 200
    assert result[5] == result[6] == {'value': False}


def test_future_stale_mismatched_clock_do_not_clear_state():
    mismatch = record(60, 60, epoch=2)
    mismatch['capture']['clock'] = 'steady'
    result = run(dict(ops=[record(100), record(50, 49, epoch=1), record(50, 151, epoch=1),
                          mismatch, query(100), query(10, epoch=1), query(100)]))
    assert result[1] == result[2] == {'value': False}
    assert result[3] == {'error': 'CLOCK_MISMATCH: stamp.clock/epoch'}
    assert result[4] == result[6]


def test_epoch_reset_and_rollback_query_and_sensor_sort_order():
    result = run(dict(required=['z', 'scan', 'a'], ops=[record(100, sensor='z'), record(100),
        record(20, sensor='camera', epoch=1), query(20, epoch=1), query(100),
        record(30, epoch=0), query(30)]))
    assert [h['sensor_id'] for h in result[3]['value']] == ['a', 'camera', 'scan', 'z']
    assert [h['health'] for h in result[3]['value']] == ['UNAVAILABLE', 'VALID', 'UNAVAILABLE', 'UNAVAILABLE']
    assert [h['sensor_id'] for h in result[6]['value']] == ['a', 'scan', 'z']


def test_calibration_downgrade_and_duplicate_capture_rejected():
    result = run(dict(ops=[record(100, calibration=5), record(101, calibration=4),
        record(100, calibration=6), record(99, calibration=6), query(101),
        record(102, calibration=5), record(103, calibration=6), query(103)]))
    assert result[1:4] == [{'value': False}] * 3
    assert result[4]['value'][0]['calibration_epoch'] == 5
    assert result[-1]['value'][0]['calibration_epoch'] == 6


@pytest.mark.parametrize('required', [[], ['scan'], ['scan', 'camera']])
def test_empty_coverage_depth_and_required_sources(required):
    ops = [query(10), dict(op='required_valid', now=stamp(10)),
           dict(op='allows', now=stamp(10), directions=[]),
           dict(op='allows_motion', now=stamp(10), velocity=[0., 0., 0.])]
    for entry in (record(10, cones=[]), record(11, depth=False), record(12), record(12, sensor='camera')):
        ops += [entry, query(12), dict(op='required_valid', now=stamp(12)),
                dict(op='allows_motion', now=stamp(12), velocity=[.1, 0., 0.]),
                dict(op='allows_motion', now=stamp(12), velocity=[0., 0., 0.])]
    run(dict(required=required, ops=ops))


def test_continuous_coverage_catches_gap_between_sampled_directions():
    directions = [-math.pi / 4, 0., math.pi / 4]
    result = run(dict(ops=[record(10, cones=[cone(d, .02) for d in directions]),
        dict(op='allows', now=stamp(10), directions=directions),
        dict(op='allows_motion', now=stamp(10), velocity=[.1, 0., 0.])]))
    assert result[1] == {'value': True}
    assert result[2] == {'value': False}


def test_one_ulp_speed_boundary_requires_coverage_and_movement_headings():
    # Python's accurate hypot is exactly .01; libc hypot rounds one ULP below.
    # Treating this as stationary would admit motion with no depth coverage.
    velocity = [-0.006876729203900029, -0.007260206295707338, 0.]
    result = run(dict(required=[], ops=[dict(op='directions', velocity=velocity),
        dict(op='allows_motion', now=stamp(0), velocity=velocity),
        dict(op='coverage_motion', coverage=[], velocity=velocity)]))
    assert len(result[0]['value']) == 3
    assert result[1] == result[2] == {'value': False}


def test_random_nextafter_speed_boundary_preserves_motion_gate():
    rng = random.Random(202609211)
    ops = []
    for index in range(2048):
        angle = rng.uniform(-math.pi, math.pi)
        vx, vy = .01 * math.cos(angle), .01 * math.sin(angle)
        if index % 3 == 1:
            vx = math.nextafter(vx, math.inf)
        elif index % 3 == 2:
            vy = math.nextafter(vy, -math.inf)
        velocity = [vx, vy, 0.]
        ops += [dict(op='directions', velocity=velocity),
                dict(op='coverage_motion', coverage=[], velocity=velocity)]
    run(dict(required=[], ops=ops))


@pytest.mark.parametrize('gap', [0., 1e-6 - 1e-12, 1e-6 + 1e-12, .005])
def test_coverage_tolerance_rotation_wrap_and_movement_thresholds(gap):
    cones = [cone(-math.pi / 8, math.pi / 8), cone(math.pi / 8 + gap, math.pi / 8)]
    ops = [record(10, cones=cones)]
    for velocity in ([.009999999, 0., 0.], [.01, 0., 0.], [.2, 0., .02],
                     [.2, 0., .020000001], [-.2, 0., 0.], [0., 0., -.020000001]):
        ops += [dict(op='allows_motion', now=stamp(10), velocity=velocity), dict(op='directions', velocity=velocity)]
    ops += [record(11, cones=[cone(math.pi, math.pi / 4)]),
            dict(op='allows_motion', now=stamp(11), velocity=[-.2, 0., 0.]),
            dict(op='allows', now=stamp(11), directions=[-math.pi, math.pi, 3 * math.pi])]
    run(dict(ops=ops))


@pytest.mark.parametrize('timeout', [1.999e-9, 0., -1e-9, .100000001])
def test_timeout_truncation_and_invalid_health_expiry(timeout):
    run(dict(timeout=timeout, ops=[query(1), record(1), query(2), record(2, 1)]))


def test_invalid_record_preserves_python_clear_and_error_order():
    invalid = record(1, epoch=1, frame='', calibration=-1)
    result = run(dict(ops=[record(100), invalid, query(100), query(1, epoch=1)]))
    assert result[1] == {'error': 'INVALID_INPUT: frame_id'}
    assert result[2]['value'][0]['health'] == 'UNAVAILABLE'


def test_invalid_same_clock_record_preserves_previous_source_sample():
    result = run(dict(ops=[record(10, calibration=3), record(11, calibration=3, frame=''),
                          query(11), record(11, calibration=-1), query(11)]))
    assert result[1] == {'error': 'INVALID_INPUT: frame_id'}
    assert result[3] == {'value': False}
    assert result[2] == result[4]
    assert result[2]['value'][0]['stamp']['ns'] == 10


@pytest.mark.parametrize('capture', [2 ** 53 + 1, 2 ** 63 - 201])
def test_large_nanosecond_stamps_remain_exact(capture):
    result = run(dict(ops=[record(capture, capture + 100), query(capture + 100), query(capture + 101)]))
    assert result[0] == {'value': True}
    assert result[1]['value'][0]['stamp']['ns'] == capture
    assert result[1]['value'][0]['health'] == 'VALID'
    assert result[2]['value'][0]['health'] == 'STALE'


def test_camera_epoch_idempotence_changes_missing_and_snapshot():
    first = calibration()
    second = calibration(calibration_epoch=2, depth_unit_m=.002)
    result = run(dict(ops=[dict(op='calibration', camera_id='front', epoch=1),
        dict(op='calibrate', calibration=first), dict(op='calibrate', calibration=first),
        dict(op='calibrate', calibration=calibration(depth_unit_m=.002)),
        dict(op='calibration', camera_id='front', epoch=2),
        dict(op='calibrate', calibration=second), dict(op='calibrate', calibration=first),
        dict(op='calibration', camera_id='front', epoch=2)]))
    assert result[0] == {'error': "'front'"}
    assert result[3] == {'error': 'calibration epoch must increase on change'}
    assert result[-1]['value'] == second


def test_invalid_new_calibration_does_not_replace_previous_epoch():
    result = run(dict(ops=[dict(op='calibrate', calibration=calibration()),
        dict(op='calibrate', calibration=calibration(calibration_epoch=2, depth_unit_m=-1.)),
        dict(op='calibration', camera_id='front', epoch=1)]))
    assert result[1] == {'error': 'INVALID_INPUT: depth_unit_m'}
    assert result[2]['value'] == calibration()


@pytest.mark.parametrize('changes,error', [
    ({'camera_id': '', 'optical_frame': ''}, 'camera_id'),
    ({'optical_frame': ' ', 'distortion_model': ''}, 'optical_frame'),
    ({'distortion_model': '', 'image_width_px': 0}, 'distortion_model'),
    ({'image_width_px': 0, 'image_height_px': 0}, 'image_width_px'),
    ({'image_height_px': -1, 'calibration_epoch': -1}, 'image_height_px'),
    ({'calibration_epoch': -1}, 'calibration_epoch'),
    ({'intrinsic_matrix': [0., 0., 0., 0., 0., 0., 0., 0., 1.]}, 'intrinsic_matrix.pinhole'),
    ({'intrinsic_matrix': ['nan'] + [0.] * 8}, 'camera.coefficient'),
    ({'distortion_coefficients': ['inf'], 'depth_unit_m': -1.}, 'camera.coefficient'),
    ({'depth_unit_m': 0.}, 'depth_unit_m'),
    ({'depth_unit_m': 'nan'}, 'depth_unit_m'),
])
def test_invalid_camera_and_error_order(changes, error):
    result = run(dict(ops=[dict(op='calibrate', calibration=calibration(**changes)),
                          dict(op='calibration', camera_id='front', epoch=1)]))
    assert result[0] == {'error': 'INVALID_INPUT: ' + error}
    assert result[1] == {'error': "'front'"}


def test_scan_cells_invalid_beams_and_infinity():
    ops = []
    for ranges in ([], [1.], ['inf', '-inf', 'nan', .1, 10., 10.00001, .0999], [1.] * 80):
        for step in (-.1, 0., .1, math.pi):
            ops.append(dict(op='scan', ranges=ranges, parameters=[.1, 10., -.5, step, .3]))
    run(dict(ops=ops))


@pytest.mark.parametrize('seed', [41903, 99173, 71359, 20260921])
def test_randomized_stateful_sequences(seed):
    rng = random.Random(seed)
    ops = []
    current = 1000
    epoch = 0
    for _ in range(1200):
        current = max(0, current + rng.randint(-25, 40))
        if rng.random() < .04:
            epoch = rng.randrange(4)
        choice = rng.randrange(6)
        if choice < 3:
            cones = [cone(rng.uniform(-math.pi, math.pi), rng.uniform(0., math.pi)) for _ in range(rng.randrange(5))]
            ops.append(record(max(0, current + rng.randint(-130, 15)), current,
                rng.choice(['scan', 'camera', 'rear']), epoch, rng.randrange(8), cones, rng.choice([True, False])))
        elif choice == 3:
            ops.append(query(current, epoch))
        elif choice == 4:
            ops.append(dict(op='allows_motion', now=stamp(current, epoch),
                velocity=[rng.uniform(-.4, .4), rng.uniform(-.4, .4), rng.choice([0., .02, -.02, .021])]))
        else:
            ops.append(dict(op='allows', now=stamp(current, epoch), directions=[rng.uniform(-6., 6.) for _ in range(rng.randrange(5))]))
        if rng.random() < .1:
            camera = calibration(camera_id=rng.choice(['front', 'rear']), calibration_epoch=rng.randrange(8),
                depth_unit_m=rng.choice([None, .001, .002]))
            ops += [dict(op='calibrate', calibration=camera),
                    dict(op='calibration', camera_id=camera['camera_id'], epoch=rng.randrange(8))]
    run(dict(required=['scan'], ops=ops))

@pytest.mark.parametrize('dimension',[2**31-1,2**31,2**32-1])
def test_wire_uint32_calibration_dimensions(dimension):
    c=calibration(image_width_px=dimension,image_height_px=dimension)
    run(dict(ops=[dict(op='calibrate',calibration=c),dict(op='calibration',camera_id='front',epoch=1)]))

@pytest.mark.parametrize('field',['image_width_px','image_height_px'])
def test_wire_calibration_negative_dimension_error_order(field):
    c=calibration(image_width_px=2**32-1,image_height_px=2**32-1)
    c[field]=-1
    run(dict(ops=[dict(op='calibrate',calibration=c)]))
    c['camera_id']=''
    run(dict(ops=[dict(op='calibrate',calibration=c)]))


@pytest.mark.parametrize('epoch', [2**63, 2**64-1, 2**64, 2**64+1, 10**399])
def test_json_integer_health_epoch_order_expiry_and_clock_reset(epoch):
    run(dict(ops=[record(100, calibration=epoch), query(100),
                  record(101, calibration=epoch-1), query(101),
                  record(100, calibration=epoch+1), query(101),
                  record(102, calibration=epoch+1), query(202), query(203),
                  record(0, epoch=1, calibration=0), query(0, epoch=1)]))


@pytest.mark.parametrize('dimension', [2**63, 2**64-1, 2**64, 10**399])
def test_json_integer_camera_dimensions_epoch_and_registration(dimension):
    c = calibration(image_width_px=dimension, image_height_px=dimension+1, calibration_epoch=dimension)
    changed = dict(c, image_width_px=dimension+1)
    run(dict(ops=[dict(op='calibrate', calibration=c), dict(op='calibrate', calibration=c),
                  dict(op='calibration', camera_id='front', epoch=dimension),
                  dict(op='calibrate', calibration=changed),
                  dict(op='calibrate', calibration=dict(changed, calibration_epoch=dimension+1)),
                  dict(op='calibration', camera_id='front', epoch=dimension),
                  dict(op='calibration', camera_id='front', epoch=dimension+1)]))


def test_json_integer_health_negative_epoch_error_and_no_state_change():
    huge = 10**399
    run(dict(ops=[record(0, calibration=-huge), query(0),
                  record(1, calibration=huge), query(1),
                  record(2, calibration=-huge), query(2)]))
