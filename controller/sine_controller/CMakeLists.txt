add_library(sine_controller SHARED src/sine_controller.cpp)
target_include_directories(sine_controller PUBLIC "include")
target_link_libraries(sine_controller PUBLIC interfaces)