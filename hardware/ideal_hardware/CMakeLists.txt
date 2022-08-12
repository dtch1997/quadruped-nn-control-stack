add_library(ideal_hardware SHARED src/ideal_hardware.cpp)
target_include_directories(ideal_hardware PUBLIC "include")
target_link_libraries(ideal_hardware PUBLIC interfaces)