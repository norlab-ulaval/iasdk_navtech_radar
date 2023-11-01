#include <rclcpp/rclcpp.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "messages/msg/radar_fft_data_message.hpp"
#include "colossus_and_camera_subscriber_to_video.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<Colossus_and_camera_subscriber_to_video>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}