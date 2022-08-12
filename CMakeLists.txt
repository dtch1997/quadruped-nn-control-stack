cmake_minimum_required(VERSION 3.1)
project(demo_control_stack)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

add_definitions(-std=c++14)

find_package (Eigen3 3.3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Custom packages
add_subdirectory(interfaces)
add_subdirectory(hardware)
# add_subdirectory(controller)

add_executable(fake_application examples/fake_application.cpp)
target_link_libraries(
    fake_application
    interfaces
)

add_executable(unitree_application examples/unitree_application.cpp)
target_link_libraries(
    unitree_application
    interfaces
    unitree_hardware
)