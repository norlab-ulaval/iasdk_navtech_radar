cmake_minimum_required(VERSION 3.16)

project(IASDK C CXX)
# enable_testing()

set (Protobuf_USE_STATIC_LIBS ON)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

add_compile_options(
    -std=c++17
    -Wall
    -Wextra
    -pthread
    $<$<CONFIG:DEBUG>:-ggdb>
    $<$<CONFIG:DEBUG>:-O0>
    $<$<CONFIG:RELEASE>:-O3>
)

add_subdirectory(utility)
add_subdirectory(protobuf)
add_subdirectory(network)
add_subdirectory(navigation)
#add_subdirectory(unittests)

# ----------------------------------------------------------------------------------------------------------
#
add_executable(colossus_client Colossus_client_main.cpp)

set_target_properties(colossus_client
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_link_libraries(colossus_client 
    utility 
    protobuf
    networking
    navigation
)

# ----------------------------------------------------------------------------------------------------------
# 
add_executable(connection_tester connection_tester.cpp)

set_target_properties(connection_tester
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_link_libraries(connection_tester 
    utility 
    protobuf
    networking
    navigation
)

# ----------------------------------------------------------------------------------------------------------
#
add_executable(colossus_protocol_test Colossus_protocol_test.cpp)

set_target_properties(colossus_protocol_test
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_link_libraries(colossus_protocol_test 
    utility 
    protobuf
    networking
    navigation
)

# ----------------------------------------------------------------------------------------------------------
#
add_executable(navigation_client navigation_main.cpp)

set_target_properties(navigation_client
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_link_libraries(navigation_client 
    utility 
    protobuf
    networking
    navigation
)

# ----------------------------------------------------------------------------------------------------------
#
add_executable(cat240_client Cat240_client_main.cpp)

set_target_properties(cat240_client
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_link_libraries(cat240_client 
    utility 
    protobuf
    networking
    navigation
)

# ----------------------------------------------------------------------------------------------------------
#
add_executable(nmea_server NMEA_server_main.cpp)

set_target_properties(nmea_server
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_link_libraries(nmea_server 
    utility 
    protobuf
    networking
    navigation
)

# ----------------------------------------------------------------------------------------------------------
#
add_executable(nmea_client NMEA_client_main.cpp)

set_target_properties(nmea_client
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

target_link_libraries(nmea_client 
    utility 
    protobuf
    networking
    navigation
)

