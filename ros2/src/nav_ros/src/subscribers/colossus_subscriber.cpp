#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <exception>
#include <cassert>

#include "messages/msg/radar_configuration_message.hpp"
#include "messages/msg/radar_fft_data_message.hpp"
#include "colossus_subscriber.h"
#include "net_conversion.h"


Colossus_subscriber::Colossus_subscriber() : Node{ "colossus_subscriber" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber = 
    Node::create_subscription<messages::msg::RadarConfigurationMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Colossus_subscriber::configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_radar_fft_subscriber(radar_fft_queue_size);
    qos_radar_fft_subscriber.reliable();

    fft_data_subscriber =
    Node::create_subscription<messages::msg::RadarFftDataMessage>(
        "radar_data/fft_data",
        qos_radar_fft_subscriber,
        std::bind(&Colossus_subscriber::fft_data_callback, this, _1)
    );
}


void Colossus_subscriber::configuration_data_callback(const messages::msg::RadarConfigurationMessage::SharedPtr msg) const
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


void Colossus_subscriber::fft_data_callback(const messages::msg::RadarFftDataMessage::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "FFT Data Received");

    auto angle = Navtech::Networking::from_vector_to<uint64_t>(msg->angle);
    if (angle.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Angle: %f", Navtech::Networking::from_uint64_host(angle.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Angle");
    }

    auto azimuth = Navtech::Networking::from_vector_to<uint16_t>(msg->azimuth);
    if (azimuth.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Azimuth: %i", Navtech::Networking::to_uint16_host(azimuth.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Azimuth");
    }

    auto sweep_counter = Navtech::Networking::from_vector_to<uint16_t>(msg->sweep_counter);
    if (sweep_counter.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Sweep Counter: %i", Navtech::Networking::to_uint16_host(sweep_counter.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Sweep Counter");
    }

    auto ntp_seconds = Navtech::Networking::from_vector_to<uint32_t>(msg->ntp_seconds);
    if (ntp_seconds.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "NTP Seconds: %i", Navtech::Networking::to_uint32_host(ntp_seconds.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: NTP Seconds");
    }

    auto ntp_split_seconds = Navtech::Networking::from_vector_to<uint32_t>(msg->ntp_split_seconds);
    if (ntp_split_seconds.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "NTP Split Seconds: %i", Navtech::Networking::to_uint32_host(ntp_split_seconds.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: NTP SPlit Seconds");
    }

    auto data_length = Navtech::Networking::from_vector_to<uint16_t>(msg->data_length);
    if (data_length.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Data Length: %i", Navtech::Networking::to_uint16_host(data_length.value()));
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Data Length");
    }
}
