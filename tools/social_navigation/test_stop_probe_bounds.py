import math

import pytest

from tools.social_navigation.run_stop_probe import probe_corridor_offsets


def test_transform_wait_receives_late_static_frame_without_motion():
    from tools.social_navigation.run_stop_probe import wait_for_transform
    wall = [0.0]
    class MissingFrame(Exception):
        pass
    def lookup():
        if wall[0] < .3:
            raise MissingFrame('not connected yet')
        return 'complete_pose'
    assert wait_for_transform(lookup, lambda: wall.__setitem__(0, wall[0]+.1),
                              MissingFrame, timeout=1., clock=lambda: wall[0]) == 'complete_pose'
    assert .3 <= wall[0] < 1.


def test_transform_wait_has_one_wall_deadline_when_clock_is_paused():
    from tools.social_navigation.run_stop_probe import wait_for_transform
    wall = [0.0]
    class MissingFrame(Exception):
        pass
    def lookup():
        raise MissingFrame('disconnected')
    with pytest.raises(TimeoutError, match='transform'):
        wait_for_transform(lookup, lambda: wall.__setitem__(0, wall[0]+.25),
                           MissingFrame, timeout=1., clock=lambda: wall[0])
    assert wall[0] == 1.


def test_transform_wait_does_not_swallow_interrupt_or_programming_error():
    from tools.social_navigation.run_stop_probe import wait_for_transform
    for error in (KeyboardInterrupt(), ValueError('bad lookup')):
        def lookup():
            raise error
        with pytest.raises(type(error)):
            wait_for_transform(lookup, lambda: None, LookupError)


@pytest.mark.parametrize('case,expected', [({}, .6), ({'probe_corridor_m': .8}, .8),
                                         ({'probe_corridor_m': .81}, .81)])
def test_corridor_includes_exact_guard_boundary(case, expected):
    offsets = probe_corridor_offsets(case)
    assert offsets[0] == 0.0 and offsets[-1] == expected
    assert all(0.0 < right-left <= .0200000001 for left, right in zip(offsets, offsets[1:]))


@pytest.mark.parametrize('value', [True, '0.8', None, math.nan, math.inf, -.8, .59, 1.01])
def test_unbounded_or_ambiguous_corridor_rejected(value):
    with pytest.raises(ValueError):
        probe_corridor_offsets({'probe_corridor_m': value})
