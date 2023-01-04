#!/bin/sh

mkdir -p /usr/lib/systemd/system
cp /home/hugoliu/github/catkin_ws/src/VIW-Fusion/daemon_node/tools/ros_daemon_node.service /usr/lib/systemd/system
systemctl daemon-reload
systemctl status ros_daemon_node.service