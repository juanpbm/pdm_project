from setuptools import find_packages
from setuptools import setup

setup(
    name='robot_trials',
    version='0.0.0',
    packages=find_packages(
        include=('robot_trials', 'robot_trials.*')),
)
