cmake_minimum_required(VERSION 3.16)

project(sdk_protobuf C CXX)

set(protobuf_files 
    configurationdata.proto 
    discovery.proto 
    health.proto 
    healthinfo.proto 
    networkinfo.proto 
    nvramcontents.proto 
    radarmodel.proto 
    softwareversions.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${protobuf_files})

message(STATUS "Protobuf current output dir: ${CMAKE_CURRENT_BINARY_DIR}")

add_library(protobuf STATIC 
    ${PROTO_SRCS} 
    ${PROTO_HDRS}
)

target_include_directories(protobuf INTERFACE
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_include_directories(protobuf PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(protobuf ${PROTOBUF_LIBRARY})
