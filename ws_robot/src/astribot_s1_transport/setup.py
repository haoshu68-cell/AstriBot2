from setuptools import setup
from glob import glob
setup(name='astribot_s1_transport', version='0.1.0',
      packages=['astribot_s1_transport'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/astribot_s1_transport']),
                  ('share/astribot_s1_transport', ['package.xml']),
                  ('share/astribot_s1_transport/config', glob('config/*')),
                  ('share/astribot_s1_transport/launch', glob('launch/*.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='Astribot', maintainer_email='dev@astribot.com',
      description='Simulation transport task transactions and scene synchronization',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'vla_probe = astribot_s1_transport.vla_probe:main',
          'vla_replay = astribot_s1_transport.vla_replay:main',
          'vla_policy_server = astribot_s1_transport.vla_server:main',
          'camera_observer = astribot_s1_transport.camera_node:main',
          'transport_task = astribot_s1_transport.ros_backend:main']})
