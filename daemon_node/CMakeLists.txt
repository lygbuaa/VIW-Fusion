cmake_minimum_required(VERSION 2.8.3)
project(daemon_node)


set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
  catkin_python_setup()
  catkin_package()

  if(CATKIN_ENABLE_TESTING)
    roslaunch_add_file_check(launch)
    roslaunch_add_file_check(test)
  endif()

  include_directories(${catkin_INCLUDE_DIRS})

  install(PROGRAMS src/daemon_node.py
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

  if(CATKIN_ENABLE_TESTING AND NOT $ENV{CI})
    find_package(rostest REQUIRED)
  endif()

endif()
