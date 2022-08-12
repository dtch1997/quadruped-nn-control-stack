
find_package(interfaces CONFIG REQUIRED)

add_executable(example_hardware_interface example_hardware_interface.cpp)
target_link_libraries(example_position
    interfaces
)