from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'astribot_s1_perception'

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
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/.gitkeep')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='astribot-dev',
    maintainer_email='dev@astribot.local',
    description=(
        '双 Livox Mid-360 点云预处理/时间同步融合 + SLAM Toolbox 建图定位'
        '（区分仿真/实体硬件两套分支）'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'livox_preprocess_node = astribot_s1_perception.livox_preprocess_node:main',
            'livox_fusion_node = astribot_s1_perception.livox_fusion_node:main',
            'autonomous_patrol_node = astribot_s1_perception.autonomous_patrol_node:main',
            'map_domain_relay = astribot_s1_perception.map_domain_relay:main',
            'map_start_cell_check = astribot_s1_perception.map_start_cell_check:main',
            'slam_adapter_node = astribot_s1_perception.slam_adapter_node:main',
            'cloud_to_grid_node = astribot_s1_perception.cloud_to_grid_node:main',
            'map_odom_tf_node = astribot_s1_perception.map_odom_tf_node:main',
        ],
    },
)
