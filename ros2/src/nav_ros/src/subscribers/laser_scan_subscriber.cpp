#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <math.h>

#include "messages/msg/radar_configuration_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_subscriber.h"
#include "net_conversion.h"


Laser_scan_subscriber::Laser_scan_subscriber() : Node{ "laser_scan_subscriber" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber = 
    Node::create_subscription<messages::msg::RadarConfigurationMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Laser_scan_subscriber::configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_radar_laser_scan_subscriber(radar_laser_scan_queue_size);
    qos_radar_laser_scan_subscriber.reliable();

    laser_scan_subscriber =
    Node::create_subscription<sensor_msgs::msg::LaserScan>(
        "radar_data/laser_scan",
        qos_radar_laser_scan_subscriber,
        std::bind(&Laser_scan_subscriber::laser_scan_callback, this, _1)
    );
}


void Laser_scan_subscriber::configuration_data_callback(const messages::msg::RadarConfigurationMessage::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");

    auto azimuth_samples = Navtech::Networking::from_vector_to<uint16_t>(msg->azimuth_samples);
    if (azimuth_samples.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", Navtech::Networking::to_uint16_host(azimuth_samples.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Azimuth Samples");
    }

    auto encoder_size = Navtech::Networking::from_vector_to<uint16_t>(msg->encoder_size);
    if (encoder_size.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", Navtech::Networking::to_uint16_host(encoder_size.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Encoder Size");
    }

    auto bin_size = Navtech::Networking::from_vector_to<uint64_t>(msg->bin_size);
    if (bin_size.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", Navtech::Networking::from_uint64_host(bin_size.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Bin Size");
    }

    auto range_in_bins = Navtech::Networking::from_vector_to<uint16_t>(msg->range_in_bins);
    if (range_in_bins.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", Navtech::Networking::to_uint16_host(range_in_bins.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Range In Bins");
    }

    auto expected_rotation_rate = Navtech::Networking::from_vector_to<uint16_t>(msg->expected_rotation_rate);
    if (expected_rotation_rate.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", Navtech::Networking::to_uint16_host(expected_rotation_rate.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Expected Rotation Rate");
    }
}


void Laser_scan_subscriber::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "Laser Scan Received");
    time_t epoch = msg->header.stamp.sec;
    RCLCPP_INFO(Node::get_logger(), "Timestamp: %s", asctime(gmtime(&epoch)));
    RCLCPP_INFO(Node::get_logger(), "Start angle: %f", msg->angle_min * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "End angle: %f", msg->angle_max * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Angle increment: %f", msg->angle_increment * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Ranges: %li", msg->ranges.size());
    RCLCPP_INFO(Node::get_logger(), "Intensities: %li", msg->intensities.size());
}
