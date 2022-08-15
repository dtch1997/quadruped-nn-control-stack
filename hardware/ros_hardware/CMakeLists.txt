cmake_minimum_required(VERSION 3.0.2)
project(static_walker)

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  genmsg
  roscpp
  std_msgs
  sensor_msgs
  roslib
  actionlib
  actionlib_msgs
  unitree_legged_msgs
  tf_conversions 
  eigen_conversions 
)

catkin_package (
 CATKIN_DEPENDS 
 geometry_msgs 
 roscpp 
 std_msgs
 sensor_msgs
 unitree_legged_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(ros_hardware SHARED
            src/robot/gazebo_comm.cpp
            src/ros_hardware.cpp)
target_link_libraries(ros_hardware PUBLIC interfaces)