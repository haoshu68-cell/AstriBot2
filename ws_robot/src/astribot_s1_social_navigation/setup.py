from glob import glob
from setuptools import setup

package = 'astribot_s1_social_navigation'
setup(name=package, version='0.1.0', packages=[package],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package]),
                  ('share/' + package, ['package.xml', 'README.md']),
                  ('share/' + package + '/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='astribot-dev', maintainer_email='astribot-dev@astribot.com',
      description='Social navigation observation adapters', license='Apache-2.0',
      entry_points={'console_scripts': [
          'social_observer = ' + package + '.observer_node:main',
          'hunav_truth_adapter = ' + package + '.hunav_adapter:main',
          'social_scenario_provider = ' + package + '.scenario:main']})
