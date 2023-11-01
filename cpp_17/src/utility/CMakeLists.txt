cmake_minimum_required(VERSION 3.16)

project(sdk_utilities C CXX)

add_library(utility STATIC
    Active.cpp
    Log.cpp
    Option_parser.cpp
    Signal_handler.cpp
    Time_utils.cpp
    Timer.cpp
)

target_include_directories(utility INTERFACE
    ${PROJECT_SOURCE_DIR}
)

target_include_directories(utility PRIVATE
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(utility PRIVATE ${CMAKE_THREAD_LIBS_INIT})


