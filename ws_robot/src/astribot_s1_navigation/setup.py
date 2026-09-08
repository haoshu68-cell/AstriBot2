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
        # 探索专用行为树（把 FollowPath 的 controller_id 指向三段式控制器实例）。
        # 不装的话 bt_navigator 会因为找不到 xml 而让每个目标直接失败。
        (os.path.join('share', package_name, 'behavior_trees'),
         glob('behavior_trees/*.xml')),
    ],
    # 评测脚本走 setup.py 的 scripts= 而**不是** data_files。
    #
    # 两个原因：
    #  ① data_files 指向 lib/<pkg>/ 时与 --symlink-install 冲突，实测直接构建失败：
    #     "No such file or directory: install/.../lib/astribot_s1_navigation/
    #      run_speed_sweep.sh"
    #  ② scripts= 会装到 setup.cfg 里 install_scripts 指定的
    #     lib/astribot_s1_navigation，也就是 `ros2 run` 找可执行文件的地方。
    # 不写这一段的话文件只躺在 src 里，install 下永远没有它，
    # 而症状是"命令找不到"——很容易被当成环境问题。
    scripts=[
        'scripts/run_speed_sweep.sh',
        'scripts/aggregate_speed_sweep.py',
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
            'path_tracking_diagnostics_node = '
            'astribot_s1_navigation.path_tracking_diagnostics_node:main',
            'explore_metrics_recorder_node = '
            'astribot_s1_navigation.explore_metrics_recorder_node:main',
        ],
    },
)
