#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['yumi_rapid_interface'],
    package_dir={'': '.'},
    requires=['rospy', 'rospy_message_converter', 'abb_rapid_msgs', 'abb_robot_msgs',
    'abb_rapid_sm_addin_msgs', 'message_generation', 'message_runtime', 'abb_robot_driver']
)

setup(**setup_args)
