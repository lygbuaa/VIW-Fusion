#!/bin/bash

export ROS_IP=10.11.10.201
export ROS_MASTER_URI=http://${ROS_IP}:11311
export ROS_HOSTNAME=${ROS_IP}
rosrun rviz rviz -d /home/hugoliu/github/catkin_ws/src/VIW-Fusion/config/avp_rviz_config.rviz