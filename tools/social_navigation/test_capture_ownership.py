from unittest.mock import Mock

import pytest

import capture_views


def test_occluding_application_is_not_accepted_as_owned_capture(tmp_path, monkeypatch):
    capture = Mock()
    monkeypatch.setattr(capture_views, 'capture', capture)
    with pytest.raises(RuntimeError, match='active'):
        capture_views.capture_exposed({'client': 42, 'frame': 7}, tmp_path/'rviz.png',
                                      lambda: 99)
    capture.assert_not_called()


def test_focus_change_during_capture_invalidates_image(tmp_path, monkeypatch):
    output = tmp_path/'rviz.png'
    monkeypatch.setattr(capture_views, 'capture', lambda *args, **kw: output.write_bytes(b'fixture'))
    active = iter([42, 99])
    with pytest.raises(RuntimeError, match='active'):
        capture_views.capture_exposed({'client': 42, 'frame': 7}, output, lambda: next(active), visible=lambda: True)
    assert not output.exists()
    assert output.with_name('rviz_occluded.png').exists()


def test_owned_window_remains_active(tmp_path, monkeypatch):
    capture = Mock()
    monkeypatch.setattr(capture_views, 'capture', capture)
    output = tmp_path/'gazebo.png'
    capture_views.capture_exposed({'client': 42, 'frame': 7}, output, lambda: 42, scene=True, visible=lambda: True)
    capture.assert_called_once_with(7, output, scene=True)


def test_non_focusing_overlay_rejected(tmp_path, monkeypatch):
    capture = Mock()
    monkeypatch.setattr(capture_views, 'capture', capture)
    with pytest.raises(RuntimeError, match='obscured'):
        capture_views.capture_exposed({'client': 42, 'frame': 7}, tmp_path/'rviz.png',
                                      lambda: 42, visible=lambda: False)
    capture.assert_not_called()


def test_non_focusing_overlay_appears_during_capture(tmp_path, monkeypatch):
    output = tmp_path/'rviz.png'
    monkeypatch.setattr(capture_views, 'capture', lambda *args, **kw: output.write_bytes(b'fixture'))
    visible = iter([True, False])
    with pytest.raises(RuntimeError, match='obscured'):
        capture_views.capture_exposed({'client': 42, 'frame': 7}, output, lambda: 42,
                                      visible=lambda: next(visible))
    assert not output.exists()
    assert output.with_name('rviz_occluded.png').exists()
