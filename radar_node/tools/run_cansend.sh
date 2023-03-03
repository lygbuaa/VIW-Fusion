#!/bin/sh

PRJ_PATH="/home/hugoliu/github/catkin_ws/"
CAN_NAME="vcan0"
echo "send can frame to: ${CAN_NAME}"

while true
do
    sleep 0.1s
    cansend vcan0 500#0800010000000E19
    cansend vcan0 503#0124007F87F80109
    cansend vcan0 504#3E80016000000301 
    cansend vcan0 505#06D3FD7F77F70201
    cansend vcan0 506#3E80024000000101
done
