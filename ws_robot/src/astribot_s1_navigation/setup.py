from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'astribot_s1_navigation'

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
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='astribot-dev',
    maintainer_email='dev@astribot.local',
    description=(
        'Nav2 自主导航集成：controller/planner/behavior/bt_navigator + '
        '车体系→world系cmd_vel转换 + 机械臂展开限速，复用现有双雷达感知/SLAM栈'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_body_to_world_node = astribot_s1_navigation.cmd_vel_body_to_world_node:main',
            'arm_speed_limiter_node = astribot_s1_navigation.arm_speed_limiter_node:main',
        ],
    },
)
