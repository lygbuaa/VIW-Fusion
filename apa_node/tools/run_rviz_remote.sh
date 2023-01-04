#!/bin/bash

export LOCAL_IP=10.11.10.129
export MASTER_IP=10.11.10.201

export ROS_IP=${LOCAL_IP}
export ROS_HOSTNAME=${LOCAL_IP}
export ROS_MASTER_URI=http://${MASTER_IP}:11311

rosrun rviz rviz -d /home/hugoliu/github/catkin_ws/src/VIW-Fusion/config/avp_rviz_config.rviz
