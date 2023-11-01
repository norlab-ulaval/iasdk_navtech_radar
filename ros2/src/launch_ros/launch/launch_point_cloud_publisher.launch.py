from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([

    Node(
        package="nav_ros",
        parameters=["../../nav_ros/config/point_cloud_publisher.yaml"],
        executable="point_cloud_publisher"
    ),

    Node(
        package="tf2_ros",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "point_cloud"],
        executable="static_transform_publisher"
    ),

    Node(
        package="rviz2",
        arguments=["-d../../rviz_views/point_cloud_view.rviz"],
        executable="rviz2"
    )
  ])