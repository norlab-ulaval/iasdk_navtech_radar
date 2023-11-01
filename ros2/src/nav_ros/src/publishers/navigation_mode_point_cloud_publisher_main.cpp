#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "navigation_mode_point_cloud_publisher.h"

#include "Colossus_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"
#include "Peak_finder.h"

using Navtech::Networking::Colossus_protocol::Radar_client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol;
using namespace Navtech::Navigation;


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Navigation_mode_point_cloud_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    node->start();
    RCLCPP_INFO(node->get_logger(), "Radar client started");

    while (rclcpp::ok()) {
        spin(node);
    }

    RCLCPP_INFO(node->get_logger(), "Stopping radar client");
    node->stop();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}