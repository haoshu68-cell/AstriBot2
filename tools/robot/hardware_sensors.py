"""Deployment descriptions and read-only readiness checks for the hardware sensor chain."""
import json
import math
from pathlib import Path
import time

SENSOR_ROLES = {'lidar', 'slam', 'perception', 'odom', 'model'}
# Only used to recognize protected vendor processes; never sourced or launched.
LIDAR_ROOT = '/opt/astribot_ros/software/livox_ros_driver2/'
SENSOR_EXECUTABLES = {
    'voxelslam': 'slam', 'nav_prob_grid_node': 'slam', 'map_odom_tf_node': 'slam',
    'livox_ros_driver2_node': 'lidar', 'chassis_odom_node': 'odom',
    'pointcloud_slice_scan_node': 'perception',
}
SENSOR_LAUNCHES = {'hardware_livox.launch.py': 'lidar', 'voxel_slam.launch.py': 'slam',
                   'hardware_perception.launch.py': 'perception', 'state_bridge.launch.py': 'model'}


def load_config(root):
    config = json.loads((root/'tools/robot/config/deployed_sensors.json').read_text())
    for key in ('lidar_ready_timeout_sec', 'navigation_inputs_timeout_sec',
                'lidar_min_rate_hz', 'lidar_max_age_sec'):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError('Invalid sensor parameter: '+key)
    return config


def commands(root, config):
    launch = ['ros2', 'launch', 'astribot_s1_perception']
    return {
        'model': ['ros2', 'launch', 'astribot_trajectory_bridge', 'state_bridge.launch.py',
                  'feedback_source:=manufacturer'],
        'lidar': launch + ['hardware_livox.launch.py', 'publish_robot_description:=false',
                          'left_config:='+str(root/config['left_config']),
                          'right_config:='+str(root/config['right_config'])],
        'slam': launch + ['voxel_slam.launch.py', 'use_sim_time:=false', 'point_notime:=0',
                         'imu_extrinsic_tran:=-0.011,-0.02329,0.04412',
                         'save_path:='+config['save_path'],
                         'previous_map:='+config.get('previous_map', ''),
                         'mode:='+('localization' if config.get('previous_map') else 'mapping'),
                         'map_name:='+config.get('map_name', ''),
                         'save_map:='+str(int(bool(config.get('map_name'))))],
        'perception': launch + ['hardware_perception.launch.py', 'use_sim_time:=false'],
        'odom': ['ros2', 'run', 'astribot_trajectory_bridge', 'chassis_odom_node',
                 '--ros-args', '-p', 'use_sim_time:=false'],
    }


def validate_files(root, config):
    paths = [root/config['left_config'], root/config['right_config']]
    for package, executable in (('livox_ros_driver2', 'livox_ros_driver2_node'),
                                ('astribot_s1_slam', 'voxelslam'), ('astribot_s1_mapping', 'nav_prob_grid_node')):
        paths.append(root/'ws_robot/install'/package/'lib'/package/executable)
    missing = [str(p) for p in paths if not p.exists()]
    if missing:
        raise RuntimeError('Release sensor dependencies missing: '+', '.join(missing))


def role(item, project_roots):
    args = item['argv']
    if not args or any(a in ('-c', '-lc', '-s') for a in args[:3]):
        return None
    tokens = args[:2] if Path(args[0]).name.startswith(('python', 'bash')) else args[:1]
    roots = tuple(project_roots)
    owned = any(a.startswith(roots) for a in tokens) or any(
        p.startswith(roots) for p in item['prefixes'].split(':'))
    if not owned:
        return None
    names = {Path(a).name for a in tokens}
    if (item.get('session') and names & {'state_bridge_node', 'robot_state_publisher'}
            and any(p.startswith(project_roots) for p in item['prefixes'].split(':'))):
        return 'model'
    if any(a.startswith('/opt/astribot_ros/') for a in tokens):
        return None
    for name, kind in SENSOR_EXECUTABLES.items():
        if name in names:
            return kind
    if names & {'ros2', 'roslaunch.py'}:
        for arg in args[2:]:
            if ':=' not in arg and Path(arg).name in SENSOR_LAUNCHES:
                return SENSOR_LAUNCHES[Path(arg).name]


class LidarReadiness:
    def __init__(self, node, config):
        from rosidl_runtime_py.utilities import get_message
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.node = node
        self.config = config
        self.rows = {}
        self.subscriptions = []
        for topic, kind in [('/livox/lidar_left', 'sensor_msgs/msg/PointCloud2'),
                            ('/livox/lidar_right', 'sensor_msgs/msg/PointCloud2'),
                            ('/livox/imu', 'sensor_msgs/msg/Imu')]:
            self.rows[topic] = []
            self.subscriptions.append(node.create_subscription(
                get_message(kind), topic, lambda m, t=topic: self.receive(t, m),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))

    def receive(self, topic, msg):
        if hasattr(msg, 'width') and (msg.width * msg.height == 0 or not msg.data):
            return
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        self.record(topic, stamp)

    def record(self, topic, stamp):
        rows = self.rows[topic]
        if rows and stamp <= rows[-1][1]:
            return
        rows.append((time.monotonic(), stamp))
        self.rows[topic] = rows[-400:]

    def report(self):
        now = time.monotonic()
        wall_ns = self.node.get_clock().now().nanoseconds
        result = {'ready': True, 'topics': {}}
        for topic, all_rows in self.rows.items():
            rows = [r for r in all_rows if now-r[0] <= 2.0]
            span = rows[-1][0]-rows[0][0] if len(rows) > 1 else 0.0
            hz = (len(rows)-1)/span if span > 0 else 0.0
            age = (wall_ns-rows[-1][1])/1e9 if rows else None
            pubs = self.node.get_publishers_info_by_topic(topic)
            ready = (len(pubs) == 1 and span >= 1.0 and hz >= self.config['lidar_min_rate_hz']
                     and now-rows[-1][0] <= self.config['lidar_max_age_sec']
                     and -0.1 <= age <= self.config['lidar_max_age_sec'])
            result['topics'][topic] = {'unique_frames': len(rows), 'unique_hz': hz,
                                      'source_age_s': age, 'publishers': len(pubs), 'ready': ready}
            result['ready'] &= ready
        return result

    def close(self):
        for sub in self.subscriptions:
            self.node.destroy_subscription(sub)
