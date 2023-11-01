# Navtech ROS2 Driver Version History

This document is intended to provide the change history of each ROS2 project, within the Navtech ROS2 driver repository. Details will include project name, version number, release date and a list of changes.

Please see lower level README.md files, for further information regarding usage of changes.

## camera_ros

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |
| 1.0.1                    | 28/10/2022            | Minor update to create output folder for saving videos                            |

## launch_ros

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |

## messages

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |

## nav_ros

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |
| 1.1.0                    | 27/10/2021            | Added ability to run *navigation mode point cloud publisher* with onboard or local processing|
| 1.1.1                    | 11/01/2022            | Fixed issue where laser_scan_publisher and point_cloud_publisher would only use a maximumum end bin of 400|
| 1.1.2                    | 12/01/2022            | Set default for end_bin to max bins from radar, improved laser_scan_publisher efficiency|
| 1.1.3                    | 13/01/2022            | Changed laser_scan_publisher and point_cloud_publisher to ignore bins before start_bin when thresholding|
| 1.2.0                    | 28/10/2022            | Updated all radar code to use the new Navtech Radar SDK                           |
| 1.3.0                    | 29/11/2022            | Fixed issue with navigaation mode point cloud not displaying correctly after SDK update|

## rviz_views

| Version Number           | Release Date          | Change(s)                                                                         |
| :----------------------- | :-------------------: | :-------------------------------------------------------------------------------: |
| 1.0.0                    | 26/10/2021            | Initial release                                                                   |