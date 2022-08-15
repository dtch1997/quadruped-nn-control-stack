cmake_minimum_required(VERSION 3.0.2)
project(ros_hardware)

include_directories(
  include 
  /opt/ros/noetic/include 
  /opt/ros/noetic/lib
)

add_library(ros_hardware SHARED
  src/robot/gazebo_comm.cpp
  src/ros_hardware.cpp
)

target_link_libraries(ros_hardware PUBLIC 
  interfaces
  -I/opt/ros/noetic/include -L/opt/ros/noetic/lib
  -lroscpp -lrostime -lrosconsole -lroscpp_serialization
)