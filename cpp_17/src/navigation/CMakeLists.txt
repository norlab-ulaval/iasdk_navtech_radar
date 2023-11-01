cmake_minimum_required(VERSION 3.16)

project(sdk_navigation C CXX)

add_library(navigation STATIC 
    Peak_finder.cpp
    Sector_blanking.cpp
)

target_include_directories(navigation INTERFACE
    ${PROJECT_SOURCE_DIR}
)

target_include_directories(navigation PRIVATE
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(navigation 
    utility 
    protobuf 
    networking
)
