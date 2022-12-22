cmake_minimum_required(VERSION 2.8.3)
project(apa_node)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport
        rosbag)
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

find_package(Ceres REQUIRED)

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
set(ORT_BASE_DIR ${PROJECT_SOURCE_DIR}/../psd/ort)
set(ORT_LIB ${ORT_BASE_DIR}/onnxruntime-linux-x64-1.13.1/lib/libonnxruntime.so)
set(PSD_BASE_DIR ${PROJECT_SOURCE_DIR}/../psd)

MESSAGE(STATUS "onnxruntime lib path: " ${ORT_LIB})
MESSAGE(STATUS "psd base dir: " ${PSD_BASE_DIR})

find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${ORT_BASE_DIR}/onnxruntime-linux-x64-1.13.1/include
  ${PSD_BASE_DIR}/inc
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES vins_lib
  CATKIN_DEPENDS roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    image_transport
        rosbag
)

add_executable(apa_node_bin src/apa_node.cc)
target_link_libraries(apa_node_bin ${catkin_LIBRARIES} ${OpenCV_LIBS} ${ORT_LIB})

