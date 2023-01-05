cmake_minimum_required(VERSION 3.10.0)
project(dataset_maker_node)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
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

include_directories(
  ${CERES_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../common/
  ${PROJECT_SOURCE_DIR}/inc/
  )
#template sophus
#set(Sophus_DIR "/home/td/slam/install/sophus/share/sophus/cmake/")
# find_package(Sophus REQUIRED)
# include_directories(/home/td/slam/install/sophus/include)

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
)

add_executable(dataset_maker_node_bin src/dataset_maker_node.cc)
target_link_libraries(dataset_maker_node_bin ${catkin_LIBRARIES} ${OpenCV_LIBS})
