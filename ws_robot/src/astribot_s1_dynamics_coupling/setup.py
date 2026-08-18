from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'astribot_s1_dynamics_coupling'

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
        '臂-底盘动力学耦合动态调速：根据机械臂展开幅度/运动速率实时限制底盘速度，'
        '无侵入中间控制层，解决机械臂运动诱发的倾倒问题'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_chassis_speed_coupling_node = '
            'astribot_s1_dynamics_coupling.arm_chassis_speed_coupling_node:main',
        ],
    },
)
