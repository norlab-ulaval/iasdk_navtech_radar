# launch_ros

The launch_ros ROS package contains example *launch files*, which will *launch all required components* for
running some of the main ROS SDK functionality. For example running the laser scan publisher, along with rviz
(with rviz config) and the laser scan to rviz transform. The launch files have been designed so that running
them will provide almost instant visual output.

## configuration options

For configuration settings, please see the README for the specific project to which the launch file corresponds.
Either the camera_ros or nav_ros folder.

## launch_ros/launch
The launch folder contains examples of the *actual .launch files*. These can be run (from the launch
directory) like so:

```
ros2 launch launch_b_scan_publisher.launch.py
```

### launch_b_scan_publisher.launch

A launch file which will start the *b_scan_publisher* executable (from the nav_ros package), along with a
*static transform*, and *RVIZ visualisation*

### launch_camera_publisher.launch

A launch file which will start the *camera_publisher* executable (from the camera_ros package), along with a
*static transform*, and *RVIZ visualisation*

### launch_colossus_and_camera_publisher.launch

A launch file which will start the *colossus_and_camera_publisher* executable (from the nav_ros package),
along with a *static transform*, and *RVIZ visualisation*

### launch_colossus_publisher.launch

A launch file which will start the *colossus_publisher* executable (from the nav_ros package), along with a
*static transform*, and *RVIZ visualisation*

### launch_laser_scan_publisher.launch

A launch file which will start the *laser_scan_publisher* executable (from the nav_ros package), along with
a *static transform*, and *RVIZ visualisation*

### launch_navigation_mode_point_cloud_publisher.launch

A launch file which will start the *navigation_mode_point_cloud_publisher* executable (from the nav_ros package),
along with a *static transform*, and *RVIZ visualisation*

### launch_point_cloud_publisher.launch

A launch file which will start the *point_cloud_publisher* executable (from the nav_ros package), along with a
*static transform*, and *RVIZ visualisation*

## CMakeLists.txt

This is the *cmake build file* which defines how the nav_ros ROS package is built and installed.

## package.xml

This file contains properties of the launch_ros package, such as package name, versions, authors etc.

Note - these properties are not intended to be edited by the user, these are *build related properties*

# Project Build instructions

Both the IASDK and the ROS2 SDK, with all prerequisites, must be installed before building this package.

See higher level README.md files for IASDK and ROS2 SDK install instructions

To build and install all ROS2 pacakges, run the following commands from the ROS2 folder:

```bash
colcon build

. install/setup.bash
```

To build and install this package only, run the following commands from the ROS2 folder:

```
colcon build --packages-select launch_ros

. install/setup.bash
```