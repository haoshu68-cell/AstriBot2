"""Exact cell identity comparison with the previous scalar scan path."""
import math
import numpy as np
import pytest
from astribot_s1_robot_geometry._geometry_native import scan_occupied_cells


def reference(ranges, lo, hi, start, step, track, map_tf, info, mask, resolution):
    def point(x, y, q):
        tx, ty, tz = -2*q[5]*y, 2*q[5]*x, 2*(q[3]*y-q[4]*x)
        return (x+q[6]*tx+q[4]*tz-q[5]*ty+q[0],
                y+q[6]*ty+q[5]*tx-q[3]*tz+q[1])
    cells = set()
    c, s, ox, oy, grid = info
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or not lo <= r < hi-.05:
            continue
        angle = start+i*step
        x, y = r*math.cos(angle), r*math.sin(angle)
        mx, my = point(x, y, map_tf)
        ix = math.floor((c*(mx-ox)+s*(my-oy))/grid)+2
        iy = math.floor((-s*(mx-ox)+c*(my-oy))/grid)+2
        if 0 <= iy < mask.shape[0] and 0 <= ix < mask.shape[1] and mask[iy, ix]:
            continue
        x, y = point(x, y, track)
        cells.add((math.floor(x/resolution), math.floor(y/resolution)))
    return np.asarray(sorted(cells), dtype=np.int64).reshape(-1, 2)


def test_rotated_maps_extrinsics_negative_cells_and_invalid_returns():
    rng = np.random.default_rng(19374)
    for _ in range(80):
        transforms = []
        for j in range(2):
            q = rng.normal(size=4);q /= np.linalg.norm(q)
            transforms.append(np.r_[rng.uniform(-2, 2, 3), q])
        ranges = rng.uniform(.05, 20., 723)
        ranges[::37] = np.nan;ranges[1::41] = np.inf;ranges[2::43] = -np.inf
        theta = rng.uniform(-math.pi, math.pi)
        info = (math.cos(theta), math.sin(theta), -3., -3., .05)
        mask = rng.random((100, 110)) < .25
        args = (ranges, .1, 20., -math.pi, 2*math.pi/723, *transforms, info, mask, .05)
        np.testing.assert_array_equal(scan_occupied_cells(*args), reference(*args))


def test_exact_range_bounds_empty_output_and_duplicate_cells():
    tf = (0., 0., 0., 0., 0., 0., 1.)
    args = ([.099, .1, 9.949, 9.95, 10., np.inf, np.nan], .1, 10., 0., .001,
            tf, tf, (1., 0., 0., 0., .1), np.zeros((10, 10), bool), .05)
    np.testing.assert_array_equal(scan_occupied_cells(*args), reference(*args))
    args = ([np.nan, np.inf], *args[1:])
    assert scan_occupied_cells(*args).shape == (0, 2)


@pytest.mark.parametrize('kind', ['quaternion', 'resolution', 'overflow'])
def test_invalid_geometry_is_rejected(kind):
    tf = [0., 0., 0., 0., 0., 0., 1.]
    resolution = .05
    if kind == 'quaternion':tf[6] = 0.
    elif kind == 'resolution':resolution = 0.
    else:tf[0] = 1e300
    with pytest.raises(ValueError):
        scan_occupied_cells([1.], .1, 10., 0., .1, tf, tf,
                            (1., 0., 0., 0., .1), np.zeros((4, 4), bool), resolution)
