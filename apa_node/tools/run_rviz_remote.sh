#!/bin/bash

export ROS_MASTER_URI=http://10.11.10.201:11311
export ROS_HOSTNAME=10.11.10.201
rosrun rviz rviz -d /home/hugoliu/github/catkin_ws/src/VIW-Fusion/config/avp_rviz_config.rviz