# run apa node
1. `roslaunch apa_node apa_carla.launch`
2. `rosbag play data/apa_town04_opt_microlino_20221219.bag`

## Prerequisites
1. `sudo apt install ros-noetic-ros-base`  #ros-base
2. `sudo apt install ros-noetic-tf`
3. `sudo apt install ros-noetic-cv-bridge`
4. `sudo apt install ros-noetic-image-transport-plugins`

# debug
`export ROS_MASTER_URI=http://10.11.10.201:11311`
`export ROS_HOSTNAME=10.11.10.201`