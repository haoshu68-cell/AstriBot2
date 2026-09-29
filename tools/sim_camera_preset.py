"""Common simulation camera parameters; paths are relative to the preset file."""
import hashlib
from pathlib import Path

import yaml

DEFAULT_RELATIVE_PRESET = 'ws_robot/src/astribot_s1_description/config/simulation_navigation_full/launch_preset.yaml'
SWITCHES = ('use_camera', 'use_wrist_cameras', 'use_stereo_cameras',
            'use_camera_postprocess', 'use_camera_pointcloud', 'enable_depth_obstacles')
PATHS = ('camera_profile', 'torso_camera_profile', 'camera_calibration_dir', 'camera_mounts_profile')


def load_camera_preset(path):
    path = Path(path).expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text())
    except yaml.YAMLError as error:
        raise ValueError('camera preset YAML cannot be parsed') from error
    if not isinstance(data, dict) or data.get('schema_version') != 1 or not data.get('baseline_id'):
        raise ValueError('camera preset requires schema_version 1 and baseline_id')
    values = data.get('parameters', {})
    if not isinstance(values, dict) or set(values) != set(SWITCHES + PATHS):
        raise ValueError('camera preset parameter keys do not match the common launch contract')
    parameters = {}
    for key in SWITCHES:
        if type(values[key]) is not bool:
            raise ValueError('camera preset switch must be a YAML boolean: ' + key)
        parameters[key] = str(values[key]).lower()
    for key in PATHS:
        if not isinstance(values[key], str) or not values[key]:
            raise ValueError('camera preset path must be a nonempty string: ' + key)
        value = (path.parent / values[key]).resolve()
        if not (value.is_dir() if key == 'camera_calibration_dir' else value.is_file()):
            raise ValueError('camera preset path unavailable: ' + str(value))
        parameters[key] = str(value)
    return parameters, {'path': str(path), 'baseline_id': data['baseline_id'],
                        'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
