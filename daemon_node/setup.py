"""
Setup for carla_manual_control
"""
import os
from glob import glob
ROS_VERSION = int(os.environ['ROS_VERSION'])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(packages=['daemon_node'], package_dir={'': 'src'})

    setup(**d)

elif ROS_VERSION == 2:
    print("daemon_node don't support ros2!")
    pass
