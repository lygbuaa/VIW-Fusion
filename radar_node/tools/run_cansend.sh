#!/bin/sh

#sudo modprobe vcan                     #Load the vcan kernel module
#sudo ip link add dev vcan0 type vcan   #Create the virtual CAN interface
#sudo ip link set up vcan0              #Bring the virtual CAN interface online

PRJ_PATH="/home/hugoliu/github/catkin_ws/"
CAN_NAME="vcan0"
echo "send can frame to: ${CAN_NAME}"

while true
do
    sleep 0.1s
    #MR415
    cansend vcan0 500#0800010000000E19
    cansend vcan0 503#0124007F87F80109
    cansend vcan0 504#3E80016000000301 
    cansend vcan0 503#06D3FD7F77F70201
    cansend vcan0 504#3E80024000000101

    #SR439 Front-Left
    cansend vcan0 61A#01059B1000000000
    cansend vcan0 61B#023CD7EB857FE807
    cansend vcan0 61C#029CCD39CC00006C
    cansend vcan0 61D#027D8FB9CD4606A0

    #SR439 Front-Right (infact Rear-Left)
    cansend vcan0 60A#010B331000000000
    cansend vcan0 60B#0141D7DF427F87F8
    cansend vcan0 60C#019CD559CC00006C
    cansend vcan0 60D#017D0FA1C5C606A0
done
