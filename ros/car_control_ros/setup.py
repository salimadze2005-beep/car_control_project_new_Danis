from setuptools import setup
from pathlib import Path
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(
    packages=['car_control_ros', 'car_control_core'],
    package_dir={'car_control_ros': 'src/car_control_ros',
                 'car_control_core': str(Path(__file__).resolve().parents[2] / 'shared/car_control_core')}))
