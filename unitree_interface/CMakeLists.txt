cmake_minimum_required(VERSION 3.1)
project(unitree_sdk)

find_library(UNITREE_LEGGED_LIB libunitree_legged_sdk_amd64.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/)

add_library(unitree_sdk SHARED src/hardware_interface.cpp)
target_include_directories(unitree_sdk PUBLIC "include")
target_link_libraries(unitree_sdk PUBLIC ${UNITREE_LEGGED_LIB} lcm -pthread)