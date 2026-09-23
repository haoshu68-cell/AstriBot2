#!/usr/bin/env python3
"""Expose and capture only the owned Gazebo and RViz window rectangles."""
import argparse
import ctypes as c
import json
import os
from pathlib import Path
import re
import subprocess
import time

from run_regression import descendants


class XImage(c.Structure):
    _fields_ = [(name, c.c_int) for name in ('width', 'height', 'xoffset', 'format')] + [
        ('data', c.c_void_p)] + [(name, c.c_int) for name in (
            'byte_order', 'bitmap_unit', 'bitmap_bit_order', 'bitmap_pad', 'depth', 'bytes_per_line', 'bits_per_pixel')] + [
        (name, c.c_ulong) for name in ('red_mask', 'green_mask', 'blue_mask')]


def viewport_rendered(pixels):
    """Reject a black Gazebo scene even when the Qt chrome has been painted."""
    width, height = pixels.size
    viewport = pixels.crop((int(width*.05), int(height*.3), int(width*.55), int(height*.85)))
    histogram = viewport.convert('L').histogram()
    return sum(histogram[9:]) > .02 * viewport.width * viewport.height


def frame_unobscured(frame):
    from Xlib import display, X
    connection = display.Display(os.environ.get('DISPLAY', ':1'))
    try:
        children = connection.screen().root.query_tree().children
        target_index = next(index for index, child in enumerate(children) if child.id == frame)
        target = children[target_index]
        if target.get_attributes().map_state != X.IsViewable:
            return False
        region = target.get_geometry()
        for window in children[target_index+1:]:
            attributes = window.get_attributes()
            if attributes.map_state != X.IsViewable or attributes.win_class == X.InputOnly:
                continue
            other = window.get_geometry()
            if (max(region.x, other.x) < min(region.x+region.width, other.x+other.width) and
                    max(region.y, other.y) < min(region.y+region.height, other.y+other.height)):
                return False
        return True
    finally:
        connection.close()


def capture_exposed(target, output, active_client, *, scene=False, visible=None):
    visible = visible or (lambda: frame_unobscured(target['frame']))
    if active_client() != target['client']:
        raise RuntimeError('Owned window is not the active client before capture')
    if not visible():
        raise RuntimeError('Owned window is obscured before capture')
    capture(target['frame'], output, scene=scene)
    active = active_client() == target['client']
    if not active or not visible():
        if output.exists():
            output.replace(output.with_name(output.stem + '_occluded.png'))
        raise RuntimeError('Owned window is not active or is obscured after capture')


def capture(frame, output, *, scene=False):
    from PIL import Image
    x = c.CDLL('libX11.so.6')
    x.XOpenDisplay.argtypes = [c.c_char_p]; x.XOpenDisplay.restype = c.c_void_p
    x.XGetGeometry.argtypes = [c.c_void_p, c.c_ulong, c.POINTER(c.c_ulong), c.POINTER(c.c_int),
        c.POINTER(c.c_int)] + [c.POINTER(c.c_uint)] * 4
    x.XGetImage.argtypes = [c.c_void_p, c.c_ulong, c.c_int, c.c_int, c.c_uint, c.c_uint, c.c_ulong, c.c_int]
    x.XGetImage.restype = c.POINTER(XImage)
    x.XTranslateCoordinates.argtypes = [c.c_void_p,c.c_ulong,c.c_ulong,c.c_int,c.c_int,
        c.POINTER(c.c_int),c.POINTER(c.c_int),c.POINTER(c.c_ulong)]
    x.XDestroyImage.argtypes = [c.POINTER(XImage)]
    x.XCloseDisplay.argtypes = [c.c_void_p]
    display = x.XOpenDisplay(os.environ.get('DISPLAY', ':1').encode())
    if not display:
        raise RuntimeError('X display unavailable')
    root = c.c_ulong(); px = c.c_int(); py = c.c_int()
    width = c.c_uint(); height = c.c_uint(); border = c.c_uint(); depth = c.c_uint()
    x.XGetGeometry(display, frame, c.byref(root), c.byref(px), c.byref(py), c.byref(width),
                  c.byref(height), c.byref(border), c.byref(depth))
    child = c.c_ulong()
    x.XTranslateCoordinates(display,frame,root.value,0,0,c.byref(px),c.byref(py),c.byref(child))
    screen_root=c.c_ulong();sx=c.c_int();sy=c.c_int();sw=c.c_uint();sh=c.c_uint()
    x.XGetGeometry(display,root.value,c.byref(screen_root),c.byref(sx),c.byref(sy),c.byref(sw),c.byref(sh),c.byref(border),c.byref(depth))
    if px.value<0 or py.value<0 or px.value+width.value>sw.value or py.value+height.value>sh.value:
        x.XCloseDisplay(display)
        raise RuntimeError('Owned window is partly outside the visible desktop')
    # Read composited screen pixels after exposure. Naming an OpenGL window's
    # backing pixmap can still return its old startup view after a camera ACK.
    pointer = x.XGetImage(display, root.value, px.value, py.value, width.value, height.value, c.c_ulong(-1), 2)
    if not pointer or pointer.contents.bits_per_pixel != 32:
        raise RuntimeError('Unsupported or unavailable window image')
    image = pointer.contents
    pixels = Image.frombytes('RGB', (image.width, image.height), c.string_at(image.data, image.bytes_per_line * image.height),
                    'raw', 'BGRX', image.bytes_per_line)
    pixels.save(output)
    x.XDestroyImage(pointer); x.XCloseDisplay(display)
    if pixels.getbbox() is None:
        raise RuntimeError('Visible display capture is black; use an available isolated display')
    if scene and not viewport_rendered(pixels):
        raise RuntimeError('Gazebo viewport has not rendered; Qt window chrome alone is insufficient')


def main():
    from Xlib import display, Xatom, X, protocol
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--supervisor', type=int, required=True)
    args = parser.parse_args()
    owned = descendants(args.supervisor)
    connection = display.Display(os.environ.get('DISPLAY', ':1'))
    windows = connection.screen().root.get_full_property(connection.intern_atom('_NET_CLIENT_LIST'), Xatom.WINDOW)
    targets = {}
    for ident in windows.value if windows else []:
        window = connection.create_resource_object('window', int(ident))
        prop = window.get_full_property(connection.intern_atom('_NET_WM_NAME'), connection.intern_atom('UTF8_STRING'))
        name = prop.value.decode() if prop else (window.get_wm_name() or '')
        pid = window.get_full_property(connection.intern_atom('_NET_WM_PID'), Xatom.CARDINAL)
        if pid is None or int(pid.value[0]) not in owned:
            continue
        label = 'gazebo' if name == 'Gazebo' else ('rviz' if name.endswith('RViz') else None)
        if label:
            if label in targets:
                raise RuntimeError('Multiple owned windows for ' + label)
            targets[label] = dict(name=name, pid=int(pid.value[0]), client=window.id,
                                  frame=window.query_tree().parent.id)
    if set(targets) != {'gazebo', 'rviz'}:
        raise RuntimeError('Both owned Gazebo and RViz windows required: ' + str(targets))
    def expose(target):
        window = connection.create_resource_object('window', target['client'])
        window.map()
        connection.screen().root.send_event(protocol.event.ClientMessage(
            window=window.id,client_type=connection.intern_atom('_NET_ACTIVE_WINDOW'),
            data=(32,[2,X.CurrentTime,0,0,0])),
            event_mask=X.SubstructureRedirectMask|X.SubstructureNotifyMask)
        connection.create_resource_object('window', target['frame']).configure(stack_mode=X.Above)
        connection.sync()
        geometry=window.get_geometry()
        window.configure(width=max(64,geometry.width-1));connection.sync();time.sleep(.2)
        window.configure(width=geometry.width);connection.sync()
        time.sleep(1.5)

    # Expose the render window before moving its camera. A hidden renderer can
    # report a new pose while its composite pixmap still shows the startup view.
    expose(targets['gazebo'])
    camera = subprocess.run(['ign', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'ignition.msgs.GUICamera',
        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req',
        'pose: {position: {x:1,y:0,z:4},orientation:{x:0,y:.707106781187,z:0,w:.707106781187}}'],
        capture_output=True, text=True, timeout=8)
    if camera.returncode or 'true' not in camera.stdout:
        raise RuntimeError('Gazebo camera reposition failed: ' + camera.stdout + camera.stderr)
    # The service acknowledges the animation request before the camera arrives.
    # Use the GUI's actual pose and keep the overhead view above the shelves.
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        pose = subprocess.run(['timeout', '2', 'ign', 'topic', '-e', '-t', '/gui/camera/pose', '-n', '1'],
                              capture_output=True, text=True, timeout=3)
        position = re.search(r'position\s*\{([^}]*)\}', pose.stdout)
        orientation = re.search(r'orientation\s*\{([^}]*)\}', pose.stdout)
        if position and orientation:
            def fields(block):
                return {k: float(v) for k, v in re.findall(r'([xyzw]):\s*([-+\deE.]+)', block.group(1))}
            p, q = fields(position), fields(orientation)
            if (sum((p.get(k, 0.)-v)**2 for k, v in zip('xyz', (1., 0., 4.))) < .0001 and
                    abs(q.get('y', 0.)*.707106781187+q.get('w', 0.)*.707106781187) > .9999):
                (args.output/'camera_pose.txt').write_text(pose.stdout)
                break
    else:
        raise RuntimeError('GUI camera did not reach the requested overhead pose')
    for label in ('gazebo', 'rviz'):
        target = targets[label]
        expose(target)
        active=connection.screen().root.get_full_property(connection.intern_atom('_NET_ACTIVE_WINDOW'),Xatom.WINDOW)
        target['active_client_at_capture'] = int(active.value[0]) if active is not None else None
        for attempt in range(3):
            try:
                def active_client():
                    prop = connection.screen().root.get_full_property(
                        connection.intern_atom('_NET_ACTIVE_WINDOW'), Xatom.WINDOW)
                    return int(prop.value[0]) if prop is not None else None
                capture_exposed(target, args.output/(label + '.png'), active_client,
                                scene=label == 'gazebo')
                break
            except RuntimeError:
                output = args.output/(label + '.png')
                if output.exists():
                    output.replace(args.output/f'{label}_capture_attempt_{attempt+1}.png')
                if attempt == 2:
                    raise
                expose(target)
                time.sleep(2.)
    (args.output/'windows.json').write_text(json.dumps(targets, indent=2) + '\n')
    connection.close()


if __name__ == '__main__':
    main()
