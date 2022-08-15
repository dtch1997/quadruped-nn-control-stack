add_subdirectory(unitree_hardware)
add_subdirectory(ideal_hardware)

if(${BUILD_WITHOUT_ROS})
    message("Skipping ros hardware")
else()
    message("Building ros hardware")
    add_subdirectory(ros_hardware)
endif()