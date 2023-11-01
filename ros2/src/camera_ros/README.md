# camera_ros

This package contains examples based on a connection to an *RTSP camera stream*. Examples include conencting to and reading
camera images, publishing camera images, subscribing to camera images, and saving camera iamges to a video file. Sample
config file(s) are also included in the 'config' directory.

## camera_ros/config

The config folder contains an example *configuration YAML file*, which define the basic *settings* needed to to run the package executables.

## camera_ros/src

The src folder contains the actual *cpp and header source files* for the camera_ros package examples. This folder contains the subfolders 'publishers', 'subscribers' and 'common' (where applicable), to further organise the source files. The files define the following executables:

### camera_publisher

Contains a basic example of *connecting to an RTSP camera*, and *publishing* both camera configuration data and image data.

#### configuration options

The following *configuration options* are included in the .yaml settings file in the config folder. These settings can be
changed by *modifying* the values in the .yaml file, prior to launching the executable.

The following options are shown for each of the settings:

* setting name

* data type

* data example

* setting description

* how to change during execution (if applicable)

#####The following options cannot be changed during execution:

| Name           | Type      | Example                  |Description                    | How to change                                |
| :------------- | :-------: | :----------------------: | :---------------------------: | :--------------------------------------------: |
| camera_url     | string    | rtsp://<stream_username>:<stream_password>@<stream_ip_address>:<stream_port><stream_url> | *The complete URI which defines the stream, including username, password, ip address and port*| N/A |

### camera_subscriber

Contains a basic example of *receiving camera configuration data and camera image data*, from the published camera topics.

#### configuration options

This executable has no settings which can be modified

### camera_subscriber_to_video

Contains a basic example of *receiving camera configuration data and camera image data*, from the published camera topics, and
saving them to the local machine as a *video file*.

#### configuration options

This executable has no settings which can be modified

### video_capture_manager

Contains some *helper code* to manage the *connection to an RTSP camera*.

#### configuration options

This executable has no settings which can be modified

## CMakeLists.txt

This is the *cmake build file* which defines how the nav_ros ROS package is built and installed.

## package.xml

This file contains *properties* of the camera_ros package, such as package name, versions, authors etc.

Note - these properties are not intended to be edited by the user, these are *build related properties*

# Project Build instructions

Both the IASDK and the ROS2 SDK, with all prerequisites, must be installed before building this package.

See higher level README.md files for IASDK and ROS2 SDK install instructions

To build and install all ROS2 pacakges, run the following commands from the ROS2 folder:

```
colcon build

. install/setup.bash
```

To build and install this package only, run the following commands from the ROS2 folder:

```
colcon build --packages-select camera_ros

. install/setup.bash
```