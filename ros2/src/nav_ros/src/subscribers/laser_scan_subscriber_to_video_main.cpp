#include <rclcpp/rclcpp.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_subscriber_to_video.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<Laser_scan_subscriber_to_video>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}