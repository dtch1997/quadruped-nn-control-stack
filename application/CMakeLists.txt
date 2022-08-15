add_library(application SHARED src/utils.cpp)
target_include_directories(application PUBLIC "include")
target_link_libraries(application PUBLIC interfaces)