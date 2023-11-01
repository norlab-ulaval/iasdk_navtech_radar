## Define CMake minimum required version
cmake_minimum_required(VERSION 3.16)

## Define project name
project(nav_ros)

## Define the SDK path
set(SDK_PATH ${PROJECT_SOURCE_DIR}/../../../cpp_17/src)

## Add compile options
add_compile_options(
    -std=c++17
    -Wall
    -Wextra
    -pthread
    $<$<CONFIG:DEBUG>:-ggdb>
    $<$<CONFIG:DEBUG>:-O0>
    $<$<CONFIG:RELEASE>:-O3>
)

## Find required packages for build
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(messages REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.5.3 REQUIRED)

## Find and include directories
set (Protobuf_USE_STATIC_LIBS ON)
find_package(Protobuf REQUIRED)
include_directories(
  ${PROTOBUF_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS})

## Add libraries
add_subdirectory(${SDK_PATH}/utility    ${CMAKE_CURRENT_BINARY_DIR}/utility)
add_subdirectory(${SDK_PATH}/protobuf   ${CMAKE_CURRENT_BINARY_DIR}/protobuf)
add_subdirectory(${SDK_PATH}/network    ${CMAKE_CURRENT_BINARY_DIR}/network)
add_subdirectory(${SDK_PATH}/navigation ${CMAKE_CURRENT_BINARY_DIR}/navigation)

## Build executable for Colossus publisher
add_executable(colossus_publisher
src/publishers/colossus_publisher_main.cpp
src/publishers/colossus_publisher.cpp)

target_link_libraries(colossus_publisher
networking
utility
protobuf)

ament_target_dependencies(
colossus_publisher
rclcpp
std_msgs
messages)

## Build executable for Colossus subscriber
add_executable(colossus_subscriber
src/subscribers/colossus_subscriber_main.cpp
src/subscribers/colossus_subscriber.cpp)

target_link_libraries(colossus_subscriber
networking
utility)

ament_target_dependencies(
colossus_subscriber
rclcpp
std_msgs
messages)

## Build executable for Colossus subscriber to video
add_executable(colossus_subscriber_to_video
src/subscribers/colossus_subscriber_to_video_main.cpp
src/subscribers/colossus_subscriber_to_video.cpp)

target_link_libraries(colossus_subscriber_to_video
networking
utility)

ament_target_dependencies(colossus_subscriber_to_video
rclcpp
std_msgs
messages
OpenCV)

## Build executable for laser scan publisher
add_executable(laser_scan_publisher
src/publishers/laser_scan_publisher_main.cpp
src/publishers/laser_scan_publisher.cpp)

target_link_libraries(laser_scan_publisher
networking
utility
protobuf)

ament_target_dependencies(laser_scan_publisher
rclcpp
std_msgs
sensor_msgs
messages)

## Build executable for Laser scan subscriber
add_executable(laser_scan_subscriber
src/subscribers/laser_scan_subscriber_main.cpp
src/subscribers/laser_scan_subscriber.cpp)

target_link_libraries(laser_scan_subscriber
networking
utility)

ament_target_dependencies(laser_scan_subscriber
rclcpp
std_msgs
sensor_msgs
messages)

## Build executable for Laser scan subscriber to video
add_executable(laser_scan_subscriber_to_video
src/subscribers/laser_scan_subscriber_to_video_main.cpp
src/subscribers/laser_scan_subscriber_to_video.cpp)

target_link_libraries(laser_scan_subscriber_to_video
networking
utility)

ament_target_dependencies(laser_scan_subscriber_to_video
rclcpp
std_msgs
sensor_msgs
messages
OpenCV)

## Build executable for point cloud publisher
add_executable(point_cloud_publisher
src/publishers/point_cloud_publisher_main.cpp
src/publishers/point_cloud_publisher.cpp)

target_link_libraries(point_cloud_publisher
networking
utility
protobuf)

ament_target_dependencies(point_cloud_publisher
rclcpp
std_msgs
sensor_msgs
messages)

## Build executable for navigation mode point cloud publisher
add_executable(navigation_mode_point_cloud_publisher
src/publishers/navigation_mode_point_cloud_publisher_main.cpp
src/publishers/navigation_mode_point_cloud_publisher.cpp)

target_link_libraries(navigation_mode_point_cloud_publisher
networking
utility
navigation
protobuf)

ament_target_dependencies(navigation_mode_point_cloud_publisher
rclcpp
std_msgs
sensor_msgs
messages)

## Build executable for b scan publisher
add_executable(b_scan_publisher
src/publishers/b_scan_publisher_main.cpp
src/publishers/b_scan_publisher.cpp)

target_link_libraries(b_scan_publisher
networking
utility
protobuf)

ament_target_dependencies(b_scan_publisher
rclcpp
std_msgs
sensor_msgs
messages
OpenCV)

## Build executable for Colossus and camera publisher
add_executable(colossus_and_camera_publisher
src/publishers/colossus_and_camera_publisher_main.cpp
src/publishers/colossus_and_camera_publisher.cpp
../camera_ros/src/common/video_capture_manager.cpp)

target_link_libraries(colossus_and_camera_publisher
networking
utility
protobuf)

ament_target_dependencies(colossus_and_camera_publisher
rclcpp
std_msgs
sensor_msgs
messages
OpenCV)

## Build executable for Colossus and camera subscriber to video
add_executable(colossus_and_camera_subscriber_to_video
src/subscribers/colossus_and_camera_subscriber_to_video_main.cpp
src/subscribers/colossus_and_camera_subscriber_to_video.cpp)

target_link_libraries(colossus_and_camera_subscriber_to_video
networking
utility
protobuf)

ament_target_dependencies(colossus_and_camera_subscriber_to_video
rclcpp
std_msgs
sensor_msgs
messages
OpenCV)

## Build executable for Colossus subscriber laser scan publisher
add_executable(colossus_subscriber_laser_scan_publisher
src/publishers/colossus_subscriber_laser_scan_publisher_main.cpp
src/publishers/colossus_subscriber_laser_scan_publisher.cpp)

target_link_libraries(colossus_subscriber_laser_scan_publisher
networking
utility
protobuf)

ament_target_dependencies(
colossus_subscriber_laser_scan_publisher
rclcpp
std_msgs
sensor_msgs
messages)

## Build executable for Colossus subscriber point cloud publisher
add_executable(colossus_subscriber_point_cloud_publisher
src/publishers/colossus_subscriber_point_cloud_publisher_main.cpp
src/publishers/colossus_subscriber_point_cloud_publisher.cpp)

target_link_libraries(colossus_subscriber_point_cloud_publisher
networking
utility
protobuf)

ament_target_dependencies(
colossus_subscriber_point_cloud_publisher
rclcpp
std_msgs
sensor_msgs
messages)

## Install the built executables
install(TARGETS
  colossus_publisher
  colossus_subscriber
  colossus_subscriber_to_video
  laser_scan_publisher
  laser_scan_subscriber
  laser_scan_subscriber_to_video
  point_cloud_publisher
  navigation_mode_point_cloud_publisher
  b_scan_publisher
  colossus_and_camera_publisher
  colossus_and_camera_subscriber_to_video
  colossus_subscriber_laser_scan_publisher
  colossus_subscriber_point_cloud_publisher
  DESTINATION lib/${PROJECT_NAME})

## Install the package
ament_package()