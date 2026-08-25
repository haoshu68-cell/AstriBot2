from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'astribot_trajectory_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='astribot-dev',
    maintainer_email='dev@astribot.local',
    description=(
        '厂商 SDK 与 ROS2 规划栈之间的唯一桥接层。Gate 2 只做状态方向：'
        'SDK 按部件读关节状态 -> 展开成逐关节 /joint_states'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_bridge_node = '
            'astribot_trajectory_bridge.state_bridge_node:main',
            'joint_map_probe = '
            'astribot_trajectory_bridge.joint_map_probe:main',
        ],
    },
)
