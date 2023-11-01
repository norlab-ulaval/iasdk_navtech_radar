# nav_ros

This package contains examples based on a connection to a Navtech radar. Examples include conencting to and reading
radar data, publishing radar data and images, subscribing to radar data and images, and saving radar iamges to a video file. Sample
config file(s) are also included in the 'config' directory.

The nav_ros ROS package also contains an example *publisher and subscriber* which interfaces to both a *Navtech Radar and
RTSP camera* simultaneously, and in a synchronised mannor.

## nav_ros/config

The config folder contains the example *configuration YAML files*, which define the basic *settings* needed to to run the
package executables.

## nav_ros/src

The src folder contains the actual *cpp and header source files* for the nav_ros package examples. This folder contains the subfolders 'publishers', 'subscribers' and 'common' (where applicable), to further organise the source files. The files define the following executables:

### b_scan_publisher

Contains a basic example of *connecting to a Navtech radar*, and *publishing* radar data as a b-scan image. With bins represented
as pixels in the horizontal, and azimuths represented as pixels in the vertical.

#### configuration options

The following *configuration options* are included in the .yaml settings file in the config folder. These settings can be changed
by *modifying* the values in the .yaml file, prior to launching the executable.

The following options are shown for each of the settings:

* setting name

* data type

* data example

* setting description

* how to change during execution (if applicable)

#####The following options can be changed during execution:

| Name           | Type      | Example                  |Description                    | How to change                                |
| :------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| azimuth_offset | int       | <radar_azimuth_offset> | The azimuth offset for which you want to 'shift' the radar data, in order to output easily interpretable data| ros2 param set b_scan_publisher azimuth_offset 200 |
| end_azimuth    | int       | <radar_end_azimuth> | The end azimuth for which you want to stop processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set b_scan_publisher end_azimuth 300 |
| end_bin        | int       | <radar_end_bin> | The end bin for which you want to stop processing radar data, range is between 0 and the maximum number of bins produced by the radar| ros2 param set b_scan_publisher end_bin 1000 |
| start_azimuth  | int       | <radar_start_azimuth> | The start azimuth for which you want to start processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set b_scan_publisher start_azimuth 100 |
| start_bin      | int       | <radar_start_bin> | The start bin for which you want to start processing radar data, range is between 0 and the maximum number of bins produced by the radar | ros2 param set b_scan_publisher start_bin 500|

#####The following options cannot be changed during execution:

| Name           | Type      | Example                  |Description                    | How to change                                |
| :------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| radar_ip       | string    | <radar_ip_address>       | The IP address of the radar you are connecting to| N/A |
| radar_port     | string    | <radar_port>             | The port number of the radar you are connecting to| N/A |

### colossus_and_camera_publisher

Contains a basic example of *connecting to a Navtech radar and an RTSP camera*, and *publishing* radar fft data and camer image data,
in a synchronised mannor.

#### configuration options

The following *configuration options* are included in the .yaml settings file in the config folder. These settings can be changed
by *modifying* the values in the .yaml file, prior to launching the executable.

The following options are shown for each of the settings:

setting name

data type

setting description

how to change during execution (if applicable)

#####The following options cannot be changed during execution:

| Name           | Type      | Example                  |Description                    | How to change                                |
| :------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| camera_url     | string    | <rtsp://<stream_username>:<stream_password>@<stream_ip_address>:<stream_port><stream_url>> | The IP address of the radar you are connecting to| N/A |
| radar_ip       | string    | <radar_ip_address> | The IP address of the radar you are connecting to| N/A |
| radar_port     | string    | <radar_port> | The port number of the radar you are connecting to| N/A |

### colossus_and_camera_subscriber_to_video

Contains a basic example of *subscribing to radar and camera topics*, and *saving* the published topics as video data.

#### configuration options

This executable currently has no configurable settings

### colossus_publisher

Contains a basic example of *connecting to a Navtech radar*, and *publishing* radar configuration and radar fft data.

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
| :------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| radar_ip       | string    | <radar_ip_address>       | The IP address of the radar you are connecting to| N/A |
| radar_port     | string    | <radar_port>             | The port number of the radar you are connecting to| N/A |

### colossus_subscriber

Contains a basic example of *subscribing to radar topics*, and *displaying* basic radar configurration information.

#### configuration options

This executable currently has no configurable settings

### colossus_subscriber_to_video

Contains a basic example of *subscribing to radar topics*, and *saving* the published topic as video data.

#### configuration options

This executable currently has no configurable settings

#### configuration options

The following *configuration options* are included in the .yaml settings file in the config folder. These settings can be changed
by *modifying* the values in the .yaml file, prior to launching the executable.

The following options are shown for each of the settings:

* setting name

* data type

* data example

* setting description

* how to change during execution (if applicable)

#####The following options cannot be changed during execution:

| Name           | Type      | Example                  |Description                    | How to change                                |
| :------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| radar_ip       | string    | <radar_ip_address>       | The IP address of the radar you are connecting to| N/A |
| radar_port     | string    | <radar_port>             | The port number of the radar you are connecting to| N/A |

### laser_scan_publisher

Contains a basic example of *connecting to a Navtech Radar*, and *publishing* radar data, as a ROS laser scan.

See the ROS laser scan message definition for information about the message type.

#### configuration options

The following *configuration options* are included in the .yaml settings file in the config folder. These settings can be changed
by *modifying* the values in the .yaml file, prior to launching the executable.

The following options are shown for each of the settings:

* setting name

* data type

* data example

* setting description

* how to change during execution (if applicable)

#####The following options can be changed during execution:

| Name            | Type      | Example                  |Description                    | How to change                                |
| :-------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| azimuth_offset  | int       | <radar_azimuth_offset> | The azimuth offset for which you want to 'shift' the radar data, in order to output easily interpretable data| ros2 param set laser_scan_publisher azimuth_offset 200 |
| end_azimuth     | int       | <radar_end_azimuth> | The end azimuth for which you want to stop processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set laser_scan_publisher end_azimuth 300 |
| end_bin         | int       | <radar_end_bin> | The end bin for which you want to stop processing radar data, range is between 0 and the maximum number of bins produced by the radar| ros2 param set laser_scan_publisher end_bin 1000 |
| start_azimuth   | int       | <radar_start_azimuth> | The start azimuth for which you want to start processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set laser_scan_publisher start_azimuth 100 |
| start_bin       | int       | <radar_start_bin> | The start bin for which you want to start processing radar data, range is between 0 and the maximum number of bins produced by the radar | ros2 param set laser_scan_publisher start_bin 500|
| power threshold | int       | <radar_return_power_threshold> | Any return power below this threshold is discarded, and any equal to or above is used to produce a laser scan point. Range is between 0 and the maximum return power (in half DB steps) of the radar| ros2 param set laser_scan_publisher power_threshold 110 |

#####The following options cannot be changed during execution:

| Name           | Type      | Example                  |Description                    | How to change                                |
| :------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| radar_ip       | string    | <radar_ip_address>       | The IP address of the radar you are connecting to| N/A |
| radar_port     | string    | <radar_port>             | The port number of the radar you are connecting to| N/A |

### colossus_subscriber_laser_scan_publisher

Contains a basic example of *subscribing to colossus ROS messages*, and *publishing* radar data, as a ROS laser scan.

See the ROS laser scan message definition for information about the message type.

#### configuration options

The following *configuration options* are included in the .yaml settings file in the config folder. These settings can be changed
by *modifying* the values in the .yaml file, prior to launching the executable.

The following options are shown for each of the settings:

* setting name

* data type

* data example

* setting description

* how to change during execution (if applicable)

#####The following options can be changed during execution:

| Name            | Type      | Example                  |Description                    | How to change                                |
| :-------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| azimuth_offset  | int       | <radar_azimuth_offset> | The azimuth offset for which you want to 'shift' the radar data, in order to output easily interpretable data| ros2 param set colossus_subscriber_laser_scan_publisher azimuth_offset 200 |
| end_azimuth     | int       | <radar_end_azimuth> | The end azimuth for which you want to stop processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set colossus_subscriber_laser_scan_publisher end_azimuth 300 |
| end_bin         | int       | <radar_end_bin> | The end bin for which you want to stop processing radar data, range is between 0 and the maximum number of bins produced by the radar| ros2 param set colossus_subscriber_laser_scan_publisher end_bin 1000 |
| start_azimuth   | int       | <radar_start_azimuth> | The start azimuth for which you want to start processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set colossus_subscriber_laser_scan_publisher start_azimuth 100 |
| start_bin       | int       | <radar_start_bin> | The start bin for which you want to start processing radar data, range is between 0 and the maximum number of bins produced by the radar | ros2 param set colossus_subscriber_laser_scan_publisher start_bin 500|
| power threshold | int       | <radar_return_power_threshold> | Any return power below this threshold is discarded, and any equal to or above is used to produce a laser scan point. Range is between 0 and the maximum return power (in half DB steps) of the radar| ros2 param set colossus_subscriber_laser_scan_publisher power_threshold 110 |
| range_offset    | double    | <range_offset>    | The range offset to add to the calculated laser scan points | ros2 param set colossus_subscriber_laser_scan_publisher range_offset 0.31 |

### laser_scan_subscriber

Contains a basic example of *subscribing to a ROS laser scan*, and *displaying* basic laser scan information.

#### configuration options

This executable currently has no configurable settings

### laser_scan_subscriber_to_video

Contains a basic example of *subscribing to a ROS laser scan*, and *saving*  the published topic as video data.

#### configuration options

This executable currently has no configurable settings

### navigation_mode_point_cloud_publisher

Contains a basic example of *connecting to a Navtech Radar*, running the radar in navigation mode, and *publishing* navigation
radar data, as a ROS point cloud.

See the ROS point cloud message definition for information about the message type.

#### configuration options

The following *configuration options* are included in the .yaml settings file in the config folder. These settings can be
changed by *modifying* the values in the .yaml file, prior to launching the executable.

The following options are shown for each of the settings:

* setting name

* data type

* data example

* setting description

* how to change during execution (if applicable)

#####The following options can be changed during execution:

| Name                  | Type      | Example                  |Description                    | How to change                                |
| :-------------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| azimuth_offset        | int        | <radar_azimuth_offset> | The azimuth offset for which you want to 'shift' the radar data, in order to output easily interpretable data| ros2 param set navigation_mode_point_cloud_publisher azimuth_offset 200 |
| end_azimuth           | int        | <radar_end_azimuth> | The end azimuth for which you want to stop processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set navigation_mode_point_cloud_publisher end_azimuth 300 |
| end_bin               | int        | <radar_end_bin> | The end bin for which you want to stop processing radar data, range is between 0 and the maximum number of bins produced by the radar| ros2 param set navigation_mode_point_cloud_publisher end_bin 1000 |
| start_azimuth         | int        | <radar_start_azimuth> | The start azimuth for which you want to start processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set navigation_mode_point_cloud_publisher start_azimuth 100 |
| start_bin             | int        | <radar_start_bin> | The start bin for which you want to start processing radar data, range is between 0 and the maximum number of bins produced by the radar | ros2 param set navigation_mode_point_cloud_publisher start_bin 500|
| power threshold       | double     | <radar_return_power_threshold> | The number of bins to run the peak finding algorithm on.| ros2 param set laser_scan_publisher power_threshold 110.0 |
| bins_to_operate_on    | int        | <number_of_bins> | The start bin for which you want to start processing radar data, range is between 0 and the maximum number of bins produced by the radar | ros2 param set navigation_mode_point_cloud_publisher start_bin 500|
| max_peaks_per_azimuth | int        | <number_of_azimuths> | The maximum number of peaks to return for any single azimuth that the peak finding algorithm is run on. | ros2 param set navigation_mode_point_cloud_publisher max_peaks_per_azimuth 3|

#####The following options cannot be changed during execution:

| Name           | Type      | Example                  |Description                    | How to change                                |
| :------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| radar_ip       | string    | <radar_ip_address>       | The IP address of the radar you are connecting to| N/A |
| radar_port     | string    | <radar_port>             | The port number of the radar you are connecting to| N/A |
| process_locally| bool      | <process_locally>        | false to process navigation data on the radar, true to process locally| N/A |

### point_cloud_publisher

Contains a basic example of *connecting to a Navtech Radar*, and *publishing* radar data, as a ROS point cloud.

See the ROS point cloud message definition for information about the message type.

#### configuration options

The following *configuration options* are included in the .yaml settings file in the config folder. These settings can be
changed by *modifying* the values in the .yaml file, prior to launching the executable.

The following options are shown for each of the settings:

* setting name

* data type

* data example

* setting description

* how to change during execution (if applicable)

#####The following options can be changed during execution:

| Name            | Type      | Example                  |Description                    | How to change                                |
| :-------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| azimuth_offset  | int       | <radar_azimuth_offset> | The azimuth offset for which you want to 'shift' the radar data, in order to output easily interpretable data| ros2 param set point_cloud_publisher azimuth_offset 200 |
| end_azimuth     | int       | <radar_end_azimuth> | The end azimuth for which you want to stop processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set point_cloud_publisher end_azimuth 300 |
| end_bin         | int       | <radar_end_bin> | The end bin for which you want to stop processing radar data, range is between 0 and the maximum number of bins produced by the radar| ros2 param set point_cloud_publisher end_bin 1000 |
| start_azimuth   | int       | <radar_start_azimuth> | The start azimuth for which you want to start processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set point_cloud_publisher start_azimuth 100 |
| start_bin       | int       | <radar_start_bin> | The start bin for which you want to start processing radar data, range is between 0 and the maximum number of bins produced by the radar | ros2 param set point_cloud_publisher start_bin 500|
| power threshold | double    | <radar_return_power_threshold> | The number of bins to run the peak finding algorithm on.| ros2 param set laser_scan_publisher power_threshold 110.0 |

#####The following options cannot be changed during execution:

| Name           | Type      | Example                  |Description                    | How to change                                |
| :------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| radar_ip       | string    | <radar_ip_address>       | The IP address of the radar you are connecting to| N/A |
| radar_port     | string    | <radar_port>             | The port number of the radar you are connecting to| N/A |

### colossus_subscriber_point_cloud_publisher

Contains a basic example of *subscribing to colossus messages*, and *publishing* radar data, as a ROS point cloud.

See the ROS point cloud message definition for information about the message type.

#### configuration options

The following *configuration options* are included in the .yaml settings file in the config folder. These settings can be
changed by *modifying* the values in the .yaml file, prior to launching the executable.

The following options are shown for each of the settings:

* setting name

* data type

* data example

* setting description

* how to change during execution (if applicable)

#####The following options can be changed during execution:

| Name            | Type      | Example                  |Description                    | How to change                                |
| :-------------- | :-------: | :----------------------: | :---------------------------: | :------------------------------------------: |
| azimuth_offset  | int       | <radar_azimuth_offset> | The azimuth offset for which you want to 'shift' the radar data, in order to output easily interpretable data| ros2 param set colossus_subscriber_point_cloud_publisher azimuth_offset 200 |
| end_azimuth     | int       | <radar_end_azimuth> | The end azimuth for which you want to stop processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set colossus_subscriber_point_cloud_publisher end_azimuth 300 |
| end_bin         | int       | <radar_end_bin> | The end bin for which you want to stop processing radar data, range is between 0 and the maximum number of bins produced by the radar| ros2 param set colossus_subscriber_point_cloud_publisher end_bin 1000 |
| start_azimuth   | int       | <radar_start_azimuth> | The start azimuth for which you want to start processing radar data, range is between 0 and the maximum number of azimuths produced by the radar| ros2 param set colossus_subscriber_point_cloud_publisher start_azimuth 100 |
| start_bin       | int       | <radar_start_bin> | The start bin for which you want to start processing radar data, range is between 0 and the maximum number of bins produced by the radar | ros2 param set colossus_subscriber_point_cloud_publisher start_bin 500|
| power threshold | double    | <radar_return_power_threshold> | The number of bins to run the peak finding algorithm on.| ros2 param set laser_scan_publisher power_threshold 110.0 |
| combined_distance_offset | double    | <combined_distance_offset> | An offset to apply before calculating the X and Y point cloud points| ros2 param set colossus_subscriber_point_cloud_publisher combined_distance_offset 0.5 |
| combined_distance_scale_factor | double    | <combined_distance_scale_factor> | A distance scale factor to apply before calculating the X and Y point cloud points| ros2 param set colossus_subscriber_point_cloud_publisher combined_distance_scale_factor 0.9999 |
| x_distance_offset | double    | <x_distance_offset> | An X distance offset to apply to the calculated X point cloud point| ros2 param set colossus_subscriber_point_cloud_publisher x_distance_offset -0.31 |
| y_distance_offset | double    | <y_distance_offset> | An Y distance offset to apply to the calculated Y point cloud point| ros2 param set colossus_subscriber_point_cloud_publisher y_distance_offset 0.25 |

## CMakeLists.txt

This is the *cmake build file* which defines how the nav_ros ROS package is built and installed.

## package.xml

This file contains *properties* of the nav_ros package, such as package name, versions, authors etc.

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

```bash
colcon build --packages-select nav_ros

. install/setup.bash
```