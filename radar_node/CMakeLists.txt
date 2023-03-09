cmake_minimum_required(VERSION 2.8.3)
project(radar_node)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport
        rosbag)
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

set(USBCAN_LIB  ${PROJECT_SOURCE_DIR}/usbcan/controlcan/libcontrolcan.so)
message(WARNING "USBCAN_LIB: ${USBCAN_LIB}")

add_message_files(
  DIRECTORY msg
  FILES
  RadarTarget.msg
  RadarDetection.msg
)
generate_messages(
  DEPENDENCIES 
  std_msgs 
  sensor_msgs 
  visualization_msgs 
  geometry_msgs
)

include_directories(
  ${PROJECT_SOURCE_DIR}/../common/
  ${PROJECT_SOURCE_DIR}/inc/
  ${PROJECT_SOURCE_DIR}
  )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

EXECUTE_PROCESS(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
MESSAGE(STATUS "cpu architecture: ${ARCHITECTURE}")


find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES vins_lib
  CATKIN_DEPENDS roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport
    rosbag
    message_runtime
)

add_executable(radar_node_bin src/radar_node.cc)
target_link_libraries(radar_node_bin 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
  ${USBCAN_LIB}
)
