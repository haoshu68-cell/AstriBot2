from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'astribot_s1_chassis_effort_drive'

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
        '麦克纳姆底盘力矩闭环驱动：/cmd_vel经逆解+轮速PID+摩擦前馈算出effort力矩，'
        '替代VelocityControl+MecanumDrive双运动学并行架构'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_effort_drive_node = '
            'astribot_s1_chassis_effort_drive.mecanum_effort_drive_node:main',
        ],
    },
)
