cmake_minimum_required(VERSION 3.1)
project(nn_locomotion)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# set default build type
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE RelWithDebInfo)
ENDIF(NOT CMAKE_BUILD_TYPE)

add_definitions(-std=c++14)

find_package (Eigen3 3.3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
        include
        include/controller/
        include/types/
        ${EIGEN3_INCLUDE_DIR}
)

file(GLOB_RECURSE utils_sources "src/utils/*.cpp")
add_library(utils SHARED ${utils_sources} )

file(GLOB_RECURSE estimator_src "src/estimator/*.cpp")
add_library(estimator SHARED ${estimator_src} )

file(GLOB_RECURSE controller_src "src/controller/*.cpp")
add_library(controller SHARED ${controller_src})

add_subdirectory(unitree_interface)

add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position
        unitree_sdk
        utils
        controller
        estimator
        yaml-cpp
)