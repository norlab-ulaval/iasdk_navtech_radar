from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([

    Node(
        package="nav_ros",
        parameters=["../../nav_ros/config/laser_scan_publisher.yaml"],
        executable="laser_scan_publisher"
    ),

    Node(
        package="tf2_ros",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "laser_frame"],
        executable="static_transform_publisher"
    ),

    Node(
        package="rviz2",
        arguments=["-d../../rviz_views/laser_scan_view.rviz"],
        executable="rviz2"
    )
  ])