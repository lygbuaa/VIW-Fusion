# install carla_ros_bridge
1. https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/#b-using-the-source-repository

# run ros_bridge with rviz: https://carla.readthedocs.io/projects/ros-bridge/en/latest/rviz_plugin/

# run carla ros_bridge
1. `roslaunch carla_ros_bridge carla_ros_bridge_viwo.launch`, this is the bridge
2. `roslaunch carla_spawn_parking carla_spawn_parking.launch`, create vehicle && sensors based on carla_spawn_parking/config/objects.json
3. `rosrun rviz rviz`, enable visualization
4. `roslaunch carla_manual_control carla_manual_control.launch`, control the vehicle, to set throttle limit: `self._control.throttle = 0.5 if keys[K_UP] or keys[K_w] else 0.0` in carla_manual_control/src/carla_manual_control/carla_manual_control.py
