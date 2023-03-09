## Prerequisites
1. `sudo apt install can-utils`  #cansend, candump

## test
1. `bash tools/run_cansend.sh`  #send pre-defined can frames

## run usbcan demo controlcan
1. start docker with usb support and privileged mode:  docker run --gpus all -it -v `pwd`:`pwd` -w `pwd` --ipc=host  -v /etc/localtime:/etc/localtime:ro -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE --network host --privileged -v /dev/bus/usb:/dev/bus/usb ros:melodic_carla_ros_bridge /bin/bash
2. `cd radar_node/usbcan/controlcan`
3. `make clean && make`
4. `sudo ./hello_cpp`
