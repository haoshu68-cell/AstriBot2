#!/usr/bin/env python3
"""Integrate camera extrinsics from camera_audit JSON into a URDF file.

Usage:
  integrate_camera_to_urdf.py --audit audit.json --urdf robot.urdf [--out merged.urdf] [--dry-run]

The script finds TF transforms in the audit JSON whose child frame looks like a camera
(contains keywords like 'camera','realsense','zed') or matches observed camera_info frames,
then inserts a `link` and a `fixed` `joint` into the target URDF with the transform as origin.
"""
import argparse
import json
import math
import sys
from xml.etree import ElementTree as ET


def quat_to_rpy(x, y, z, w):
    # Convert quaternion to roll, pitch, yaw
    # Source: conversions for quaternion to Euler (ZYX / roll-pitch-yaw)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def looks_like_camera(frame_name):
    if not frame_name:
        return False
    name = frame_name.lower()
    for k in ('camera', 'realsense', 'zed', 'rgb', 'depth'):
        if k in name:
            return True
    return False


def ensure_unique_name(root, tag, name):
    # if name exists, return False; caller can decide
    for elem in root.findall(tag):
        if elem.get('name') == name:
            return False
    return True


def insert_camera_into_urdf(urdf_path, audit, out_path=None, dry_run=False,
                           update_existing=False, xacro_out=None):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot_tag = root

    camera_frames = set()
    # collect camera frames from camera_info and images
    for info in audit.get('camera_info', {}).values():
        hdr = info.get('header', {})
        if hdr.get('frame_id'):
            camera_frames.add(hdr['frame_id'])
    for img in audit.get('images', {}).values():
        if img.get('frame_id'):
            camera_frames.add(img['frame_id'])

    transforms = audit.get('extrinsics', []) + audit.get('tf_static', [])
    added = []
    # for xacro output we collect new link/joint elements here
    xacro_root = None
    if xacro_out:
        xacro_root = ET.Element('robot')
    for tf in transforms:
        # tf structure expected like message_to_ordereddict(TransformStamped)
        header = tf.get('header', {})
        parent = header.get('frame_id')
        child = tf.get('child_frame_id') or tf.get('child_frame')
        tr = tf.get('transform') or tf.get('transform', {})
        if not child:
            continue
        if not (looks_like_camera(child) or child in camera_frames):
            continue
        # unpack translation and rotation
        t = tr.get('translation', {})
        r = tr.get('rotation', {})
        try:
            x = float(t.get('x', 0))
            y = float(t.get('y', 0))
            z = float(t.get('z', 0))
            qx = float(r.get('x', 0))
            qy = float(r.get('y', 0))
            qz = float(r.get('z', 0))
            qw = float(r.get('w', 1))
        except Exception:
            continue
        rpy = quat_to_rpy(qx, qy, qz, qw)

        link_name = child
        joint_name = f"{parent}_to_{child}_joint"

        # check if link exists
        existing_link = None
        for el in robot_tag.findall('link'):
            if el.get('name') == link_name:
                existing_link = el
                break

        # create link if missing
        if existing_link is None:
            link = ET.Element('link', attrib={'name': link_name})
            # append to robot_tag only if not xacro output mode
            if not xacro_out:
                robot_tag.append(link)
        else:
            link = existing_link

        # find existing joint that attaches this link as child
        existing_joint = None
        for j in robot_tag.findall('joint'):
            child_el = j.find('child')
            if child_el is not None and child_el.get('link') == link_name:
                existing_joint = j
                break

        origin_xyz = f"{x} {y} {z}"
        origin_rpy = f"{rpy[0]} {rpy[1]} {rpy[2]}"

        # create or update joint
        if existing_joint is None:
            joint = ET.Element('joint', attrib={'name': joint_name, 'type': 'fixed'})
            origin = ET.Element('origin')
            origin.set('xyz', origin_xyz)
            origin.set('rpy', origin_rpy)
            parent_el = ET.Element('parent'); parent_el.set('link', parent)
            child_el = ET.Element('child'); child_el.set('link', link_name)
            joint.append(origin); joint.append(parent_el); joint.append(child_el)
            if xacro_out:
                xacro_root.append(link)
                xacro_root.append(joint)
            else:
                robot_tag.append(joint)
            action = 'created'
        else:
            if not update_existing:
                action = 'skipped'
            else:
                # update origin of existing joint (or create origin if missing)
                origin = existing_joint.find('origin')
                if origin is None:
                    origin = ET.Element('origin')
                    existing_joint.insert(0, origin)
                origin.set('xyz', origin_xyz)
                origin.set('rpy', origin_rpy)
                action = 'updated'

        added.append({'parent': parent, 'child': child, 'xyz': (x, y, z), 'rpy': rpy, 'action': action})

    if dry_run:
        print(json.dumps({'urdf': urdf_path, 'added': added}, indent=2, default=str))
        return 0

    # handle xacro output
    if xacro_out:
        xacro_tree = ET.ElementTree(xacro_root)
        xacro_tree.write(xacro_out, encoding='utf-8', xml_declaration=True)
        print(f'Wrote xacro fragment with {len(added)} entries to {xacro_out}')
        return 0

    target = out_path or urdf_path
    tree.write(target, encoding='utf-8', xml_declaration=True)
    print(f'Wrote {len(added)} camera links/joints to {target}')
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--audit', required=True, help='camera_audit JSON file')
    parser.add_argument('--urdf', required=True, help='URDF file to modify')
    parser.add_argument('--out', help='Output URDF path (default: overwrite)')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--update-existing', action='store_true',
                        help='If set, update origin of existing joints instead of skipping')
    parser.add_argument('--xacro-out', help='Write camera-only xacro/URDF fragment to given path')
    args = parser.parse_args()

    try:
        with open(args.audit, 'r') as f:
            audit = json.load(f)
    except Exception as e:
        print('Failed to load audit JSON:', e, file=sys.stderr)
        return 2

    return insert_camera_into_urdf(args.urdf, audit, out_path=args.out,
                                   dry_run=args.dry_run, update_existing=args.update_existing,
                                   xacro_out=args.xacro_out)


if __name__ == '__main__':
    sys.exit(main())
