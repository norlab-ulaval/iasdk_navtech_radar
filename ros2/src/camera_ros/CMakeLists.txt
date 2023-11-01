## Define CMake minimum required version
cmake_minimum_required(VERSION 3.5)
project(camera_ros)

## Define C++ standard as C++ 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

## Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find required packages for build
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(messages REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.5.3 REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
)

## Build executable for Video capture manager
add_executable(video_capture_manager
src/common/video_capture_manager.cpp
src/common/video_capture_manager_main.cpp)

target_link_libraries(video_capture_manager)
ament_target_dependencies(
video_capture_manager
rclcpp
std_msgs
sensor_msgs
messages
OpenCV)

## Build executable for Camera publisher
add_executable(camera_publisher
src/publishers/camera_publisher_main.cpp
src/publishers/camera_publisher.cpp)

target_link_libraries(camera_publisher)

ament_target_dependencies(
camera_publisher
rclcpp
std_msgs
sensor_msgs
messages
OpenCV)

## Build executable for Camera subscriber
add_executable(camera_subscriber
src/subscribers/camera_subscriber_main.cpp
src/subscribers/camera_subscriber.cpp)

target_link_libraries(camera_subscriber)

ament_target_dependencies(camera_subscriber
rclcpp
std_msgs
sensor_msgs
messages
OpenCV)

## Build executable for Camera subscriber to video
add_executable(camera_subscriber_to_video
src/subscribers/camera_subscriber_to_video_main.cpp
src/subscribers/camera_subscriber_to_video.cpp)

target_link_libraries(camera_subscriber_to_video)

ament_target_dependencies(camera_subscriber_to_video
rclcpp
std_msgs
sensor_msgs
messages
OpenCV)

## Install the built executables
install(TARGETS
  video_capture_manager
  camera_publisher
  camera_subscriber
  camera_subscriber_to_video
  DESTINATION lib/${PROJECT_NAME})

## Install the package
ament_package()