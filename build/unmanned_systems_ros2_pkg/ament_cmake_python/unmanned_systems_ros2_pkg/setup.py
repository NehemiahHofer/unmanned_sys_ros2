from setuptools import find_packages
from setuptools import setup

setup(
    name='unmanned_systems_ros2_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('unmanned_systems_ros2_pkg', 'unmanned_systems_ros2_pkg.*')),
)
