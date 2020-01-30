#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['auv8_mission_controller_flexbe_states'],
    package_dir = {'': 'src'}
)

setup(**d)
