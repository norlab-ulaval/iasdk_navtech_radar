from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([

    Node(
        package="nav_ros",
        parameters=["../../nav_ros/config/colossus_publisher.yaml"],
        executable="colossus_publisher"
    )
  ])