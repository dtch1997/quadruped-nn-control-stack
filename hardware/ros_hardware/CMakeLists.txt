cmake_minimum_required(VERSION 3.0.2)
project(static_walker)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(RBDL COMPONENTS URDFReader REQUIRED)
find_package (Eigen3 3.3 REQUIRED)
find_package(yaml-cpp REQUIRED)

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
  include/controller/
  ${catkin_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)


file(GLOB_RECURSE utils_sources "src/utils/*.cpp")
add_library(utils SHARED ${utils_sources} )  

add_library(ros_hardware SHARED
            src/robot/gazebo_comm.cpp
            src/ros_hardware.cpp)