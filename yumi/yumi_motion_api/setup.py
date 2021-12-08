#!/usr/bin/python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['yumi_motion_api'],
    package_dir={'': '.'},
    requires=['rospy', 'moveit_core', 'moveit_commander', 'moveit_msgs', 'moveit_ros_planning_interface', 'std_msgs',
              'geometry_msgs','sensor_msgs', 'rospy_message_converter','tf','message_generation', 'message_runtime',
               'abb_robot_driver']
)

setup(**setup_args)
