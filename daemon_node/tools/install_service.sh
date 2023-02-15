#!/bin/sh

PRJ_PATH="/home/hugoliu/github/catkin_ws/"

mkdir -p /usr/lib/systemd/system
cp ${PRJ_PATH}/src/se-autodriving-apa/daemon_node/tools/ros_daemon_node.service /usr/lib/systemd/system
cp ${PRJ_PATH}/src/se-autodriving-apa/daemon_node/tools/ros_daemon_node.timer /usr/lib/systemd/system
systemctl daemon-reload
systemctl start ros_daemon_node.timer
systemctl status ros_daemon_node.service
systemctl status ros_daemon_node.timer
