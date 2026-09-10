from glob import glob
from setuptools import setup

setup(name='car_control_sim', version='0.2.0', packages=['car_control_sim'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/car_control_sim']),
                  ('share/car_control_sim', ['package.xml']),
                  ('share/car_control_sim/launch', glob('launch/*.py')),
                  ('share/car_control_sim/config', glob('config/*.json'))],
      entry_points={'console_scripts': ['controller = car_control_sim.node:main']})
