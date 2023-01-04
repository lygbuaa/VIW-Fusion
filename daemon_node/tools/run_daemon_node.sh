#!/bin/sh

ETH_NAME="enp0s31f6"
LOCAL_IP_ADDR=`ifconfig ${ETH_NAME} | grep "inet " | awk '{print $2}'`
echo "get local ip: ${LOCAL_IP_ADDR}"

# if this daemon runs with master, MASTER_IP_ADDR is LOCAL_IP_ADDR.
# if this daemon runs as client, set MASTER_IP_ADDR manually.
MASTER_IP_ADDR=${LOCAL_IP_ADDR}

export ROS_IP=${LOCAL_IP_ADDR}
export ROS_HOSTNAME=${LOCAL_IP_ADDR}
export ROS_MASTER_URI=http://${MASTER_IP_ADDR}:11311


sleep 1s
roslaunch daemon_node daemon_node.launch