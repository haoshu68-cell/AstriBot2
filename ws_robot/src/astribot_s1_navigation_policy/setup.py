from setuptools import setup

setup(
    name='astribot_s1_navigation_policy', version='0.1.0',
    packages=['astribot_s1_navigation_policy'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/astribot_s1_navigation_policy']),
        ('share/astribot_s1_navigation_policy', ['package.xml', 'README.md']),
        ('share/astribot_s1_navigation_policy/config', ['config/simulation.json', 'config/hardware.template.json', 'config/social.json',
            'config/h2_simulation.json', 'config/measured_stop_reference_20260915.json']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='astribot-dev', maintainer_email='astribot-dev@astribot.com',
    description='Sensor observation and navigation decision contracts.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'policy_controller = astribot_s1_navigation_policy.policy_node:main',
        'policy_observer = astribot_s1_navigation_policy.observer_node:main',
    ]},
)
