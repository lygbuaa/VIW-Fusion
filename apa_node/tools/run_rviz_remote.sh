#!/bin/bash

# add this route on host to acces 10.0.3.* in LXC container
# route add -net 10.0.3.0 netmask 255.255.255.0 gw 10.11.10.201

export LOCAL_IP=10.11.10.129
export MASTER_IP=10.0.3.2

export ROS_IP=${LOCAL_IP}
export ROS_HOSTNAME=${LOCAL_IP}
export ROS_MASTER_URI=http://${MASTER_IP}:11311

# rosrun rviz rviz -d /home/hugoliu/github/catkin_ws/src/VIW-Fusion/config/avp_rviz_config.rviz
rosrun rviz rviz
