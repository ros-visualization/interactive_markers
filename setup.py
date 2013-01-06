#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['interactive_markers'],
    package_dir={'': 'src'},
    requires=['rospy', 'visualization_msgs', 'std_msgs', 'roslib']
)

setup(**d)
