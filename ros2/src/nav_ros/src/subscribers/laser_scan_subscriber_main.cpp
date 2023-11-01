#include <rclcpp/rclcpp.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_subscriber.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Laser_scan_subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}