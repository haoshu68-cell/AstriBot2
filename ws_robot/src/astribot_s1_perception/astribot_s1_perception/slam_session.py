"""Unified Voxel session operations. No sensor forwarding or alternate SLAM backend."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import re

import rclpy
from astribot_logging import get_logger
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from astribot_slam_msgs.msg import KeyframePoseArray
import yaml


def inspect_session(directory, *, require_manifest=True):
    directory = Path(directory).resolve()
    yaml_path = directory / (directory.name + '.yaml')
    metadata = yaml.safe_load(yaml_path.read_text())
    image = (directory / metadata['image']).resolve()
    if image.parent != directory or image.stat().st_size < 20:
        raise ValueError('地图图像缺失或不在会话目录内')
    pgm = image.read_bytes()
    match = re.match(rb'P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+255[ \r]?\n', pgm)
    if not match or len(pgm)-match.end() != int(match[1])*int(match[2]):
        raise ValueError('地图 PGM 数据未写完整或格式非法')
    if not math.isfinite(metadata['resolution']) or metadata['resolution'] <= 0:
        raise ValueError('地图分辨率非法')
    if len(metadata['origin']) != 3 or not all(math.isfinite(v) for v in metadata['origin']):
        raise ValueError('地图原点非法')
    poses = directory / 'alidarState.txt'
    clouds = sorted((directory / 'kf').glob('*.pcd'))
    if not poses.is_file() or poses.stat().st_size == 0 or not clouds:
        raise ValueError('Voxel 会话轨迹或关键帧缺失')
    rows = poses.read_text().splitlines()
    scan_count = len(rows)
    for index, row in enumerate(rows):
        values = [float(value) for value in row.split()]
        if len(values) != 26 or not all(math.isfinite(value) for value in values):
            raise ValueError(f'无效扫描位姿行: {index}')
        if abs(sum(value*value for value in values[4:8])-1.0) > 1e-3:
            raise ValueError(f'扫描位姿四元数未归一化: {index}')
    for cloud in clouds:
        if not cloud.stem.isdigit() or int(cloud.stem) >= scan_count or cloud.stat().st_size < 100:
            raise ValueError(f'无效关键帧: {cloud}')
    files = [yaml_path, image, poses, *clouds]
    result = {'format_version': 1, 'backend': 'voxel_slam', 'session': directory.name,
            'world_frame': 'map', 'map_yaml': yaml_path.name,
            'keyframes': len(clouds), 'scans': scan_count,
            'sha256': {str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in files}}
    manifest_path = directory / 'manifest.json'
    if manifest_path.exists():
        saved = json.loads(manifest_path.read_text())
        for name in ('format_version', 'backend', 'session', 'world_frame', 'map_yaml', 'keyframes', 'scans'):
            if saved.get(name) != result[name]:
                raise ValueError(f'会话 manifest 契约不一致: {name}')
        if saved.get('sha256') != result['sha256']:
            raise ValueError('会话文件与已提交 manifest 校验和不一致')
    elif require_manifest:
        raise ValueError('会话没有完成存图提交，缺少 manifest.json')
    return result



def save_session(timeout):
    node = rclpy.create_node('slam_session_save')
    final = []
    velocity = []
    node.create_subscription(KeyframePoseArray, '/voxel_slam/keyframe_pose_array',
                             lambda m: final.append(m) if m.is_final else None, 10)
    node.create_subscription(Odometry, '/odom',
        lambda m: velocity.append((time.monotonic(), math.hypot(m.twist.twist.linear.x,
            m.twist.twist.linear.y), abs(m.twist.twist.angular.z))), qos_profile_sensor_data)
    deadline = time.monotonic() + timeout

    def wait(predicate, limit=deadline):
        while not predicate():
            if time.monotonic() >= limit:
                raise TimeoutError('等待 SLAM 保存完成超时；未提交 manifest')
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        get = node.create_client(GetParameters, '/voxelslam/get_parameters')
        wait(lambda: get.service_is_ready())
        request = GetParameters.Request(names=['General.save_path', 'General.mapname', 'General.is_save_map'])
        future = get.call_async(request)
        wait(future.done)
        values = future.result().values
        if values[2].integer_value != 1 or not values[1].string_value:
            raise RuntimeError('启动时须设置 save_map:=1 和唯一 map_name')
        directory = (Path(values[0].string_value) / values[1].string_value).resolve()
        wait(lambda: len(velocity) >= 5 and velocity[-1][0] - velocity[0][0] >= 0.5)
        recent = [v for v in velocity if time.monotonic()-v[0] < 0.5]
        if not recent or any(v[1] > .01 or v[2] > .02 for v in recent):
            raise RuntimeError('机器人尚未停稳；先结束探索/导航，再保存会话')
        setter = node.create_client(SetParameters, '/voxelslam/set_parameters')
        wait(lambda: setter.service_is_ready())
        request = SetParameters.Request(parameters=[Parameter(name='finish', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=True))])
        future = setter.call_async(request)
        wait(future.done)
        if not all(result.successful for result in future.result().results):
            raise RuntimeError('Voxel-SLAM 拒绝 finish 请求')
        get_logger('astribot.slam_session').info('FINALIZING %s', directory)
        wait(lambda: bool(final))
        event = final[-1]
        if Path(event.save_dir).resolve() != directory or event.map_name != directory.name:
            raise RuntimeError('保存完成事件与请求会话不一致')
        last_error = None
        while time.monotonic() < deadline:
            try:
                manifest = inspect_session(directory, require_manifest=False)
                manifest['final_pose_updates'] = len(event.updates)
                target = directory / 'manifest.json'
                temporary = target.with_suffix('.json.tmp')
                temporary.write_text(json.dumps(manifest, indent=2) + '\n')
                temporary.replace(target)
                get_logger('astribot.slam_session').info('SAVED %s', target)
                return
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
                last_error = exc
                rclpy.spin_once(node, timeout_sec=.1)
        raise TimeoutError(f'优化结束但会话不完整: {last_error}')
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='operation', required=True)
    save = sub.add_parser('save', help='停稳后结束当前建图、等待三维/二维文件并提交 manifest')
    save.add_argument('--timeout', type=float, default=120.0)
    check = sub.add_parser('inspect', help='离线校验完整 Voxel 会话')
    check.add_argument('directory')
    args, ros_args = parser.parse_known_args()
    if args.operation == 'inspect':
        print(json.dumps(inspect_session(args.directory), indent=2))
        return
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout 必须为有限正数')
    rclpy.init(args=ros_args)
    try:
        save_session(args.timeout)
    except (RuntimeError, TimeoutError, OSError, ValueError) as exc:
        get_logger('astribot.slam_session').exception('SLAM session save failed')
        raise SystemExit(str(exc)) from exc
    finally:
        rclpy.shutdown()
