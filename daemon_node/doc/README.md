# ros-melodic install on python2
1. so run python scripts with python2
2. send cmd to daemon_node: `rostopic pub /daemon_node/cmd std_msgs/Int32 "data: 0" -1`

# kill all process
1. `pkill -9 ros`