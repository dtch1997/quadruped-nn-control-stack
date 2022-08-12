find_library(UNITREE_LEGGED_LIB libunitree_legged_sdk_amd64.so ${CMAKE_CURRENT_SOURCE_DIR}/lib/)

add_library(unitree_hardware SHARED src/unitree_hardware.cpp)
target_include_directories(unitree_hardware PUBLIC "include")
target_link_libraries(unitree_hardware PUBLIC ${UNITREE_LEGGED_LIB} lcm -pthread interfaces)