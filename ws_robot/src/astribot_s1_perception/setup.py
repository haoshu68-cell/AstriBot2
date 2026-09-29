from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'astribot_s1_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/.gitkeep')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='astribot-dev',
    maintainer_email='dev@astribot.local',
    description='标准 PointCloud2/IMU 感知与统一 Voxel-SLAM 接入',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'slam_session = astribot_s1_perception.slam_session:main',
            'autonomous_patrol_node = astribot_s1_perception.autonomous_patrol_node:main',
            'map_start_cell_check = astribot_s1_perception.map_start_cell_check:main',
        ],
    },
)
