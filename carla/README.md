# run carla server
1. get into docker: docker run -it --privileged --gpus all --net=host -e DISPLAY=unix$DISPLAY carlasim/carla:0.9.13 /bin/bash
2. run server:      bash ./CarlaUE4.sh

# run client
1. cd VIW-Fusion/carla
2. python3 patrol_in_town04.py

# client dependencies
pip3 install carla
pip3 install pygame
pip3 install numpy

# carla map
1. localization coordinate, used by carla.Location and GNSS, X: east, Y: west, Z: up, 
2. compass and heading, 0: north, 90: east, 180: south, 270: west
3. sensor coordinate, the same as Unreal Engine,  x-forward, y-right, z-up
4. parking lot in Town04: carla.Location(x=290.0, y=-180.0, z=0.0)
5. carla.Transform() is default value, carla.Location(x=0.0, y=0.0, z=0.0), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0), this position is the center of vehicle: x is midpoint of vehicle lenght, y is midpoint of vehicle width, z is 0 on the ground.