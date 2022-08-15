add_library(ros_hardware SHARED
  src/robot/gazebo_comm.cpp
  src/ros_hardware.cpp
)
target_include_directories(ros_hardware PUBLIC "include" 
  /opt/ros/noetic/include 
  /opt/ros/noetic/lib
)
target_link_libraries(ros_hardware PUBLIC 
  interfaces
  -I/opt/ros/noetic/include -L/opt/ros/noetic/lib
  -lroscpp -lrostime -lrosconsole -lroscpp_serialization
)