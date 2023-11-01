#include <rclcpp/rclcpp.hpp>

#include "messages/msg/camera_configuration_message.hpp"
#include "camera_subscriber_to_video.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Camera_subscriber_to_video>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
