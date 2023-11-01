#include <rclcpp/rclcpp.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "messages/msg/radar_fft_data_message.hpp"
#include "colossus_subscriber.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Colossus_subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}