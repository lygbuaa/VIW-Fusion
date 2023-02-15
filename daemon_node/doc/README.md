# ros-noetic install on python3
1. so run python scripts with python3

# ros-melodic install on python2
1. so run python scripts with python2
2. send cmd to daemon_node: `rostopic pub /daemon_node/cmd std_msgs/Int32 "data: 0" -1`

# run daemon node
1. `/bin/bash /home/hugoliu/github/catkin_ws/src/se-autodriving-apa/daemon_node/tools/run_daemon_node.sh`

# kill all process
1. `pkill -9 ros`

# autostart
1. install systemd service and timer: `bash install_service.sh`