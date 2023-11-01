project(iasdk C CXX)

cmake_minimum_required(VERSION 3.5)

set (CMAKE_CXX_FLAGS " -Wall -pthread -std=c++17")
add_definitions(${CMAKE_CXX_FLAGS})

add_library(iasdk_static STATIC radarclient.cpp tcpradarclient.cpp tcpsocket.cpp timer.cpp threadedclass.cpp)
add_library(iasdk SHARED radarclient.cpp tcpradarclient.cpp tcpsocket.cpp timer.cpp threadedclass.cpp)

add_executable(testclient_static testclient_main.cpp)
target_link_libraries(testclient_static iasdk_static)

add_executable(testclient_dynamic testclient_main.cpp)
target_link_libraries(testclient_dynamic iasdk)
