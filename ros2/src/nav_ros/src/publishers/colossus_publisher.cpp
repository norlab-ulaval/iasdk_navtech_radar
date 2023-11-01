#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "messages/msg/radar_fft_data_message.hpp"
#include "colossus_publisher.h"
#include "net_conversion.h"
#include "Endpoint.h"

#include "Colossus_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

using namespace Navtech::Networking;
using Navtech::Networking::Colossus_protocol::Radar_client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol;


Colossus_publisher::Colossus_publisher():Node{ "colossus_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Navtech::Networking::Colossus_protocol::Radar_client>(
        server_addr
    );

    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&Colossus_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::configuration, 
        std::bind(&Colossus_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<messages::msg::RadarConfigurationMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher
    );

    rclcpp::QoS qos_radar_fft_publisher(radar_fft_queue_size);
    qos_radar_fft_publisher.reliable();

    fft_data_publisher =
    Node::create_publisher<messages::msg::RadarFftDataMessage>(
        "radar_data/fft_data",
        qos_radar_fft_publisher
    );
}


Colossus_publisher::~Colossus_publisher()
{
    stop();
}


void Colossus_publisher::start()
{
    radar_client->start();
}


void Colossus_publisher::stop()
{
    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);

    radar_client->send(Type::stop_fft_data);
    radar_client->stop();
}


void Colossus_publisher::fft_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;

    auto fft  = msg.view_as<Colossus_protocol::FFT_data>();
    auto data = fft->to_vector();

    auto message = messages::msg::RadarFftDataMessage();
    message.header = std_msgs::msg::Header();
    message.header.stamp = Node::get_clock()->now();
    message.angle = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(fft->azimuth() / azimuth_samples * 360.0));
    message.azimuth = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(fft->azimuth()));
    message.sweep_counter = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(fft->sweep_counter()));
    message.ntp_seconds = Navtech::Networking::to_vector(Navtech::Networking::to_uint32_network(fft->ntp_seconds()));
    message.ntp_split_seconds = Navtech::Networking::to_vector(Navtech::Networking::to_uint32_network(fft->ntp_split_seconds()));
    message.data = data;
    message.data_length = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(data.size()));

    // On every radar rotation, send out the latest camera frame
    if (fft->azimuth() < last_azimuth) {
        rotation_count++;
        rotated_once = true;
    }
    last_azimuth = fft->azimuth();

    if (rotation_count >= config_publish_count) {
        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }

    if (!rotated_once) {
        return;
    }
	
    fft_data_publisher->publish(message);
}


void Colossus_publisher::configuration_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    auto config   = msg.view_as<Configuration>();
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", config->azimuth_samples());
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", config->encoder_size());
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %i", config->bin_size());
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", config->range_in_bins());
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", config->rotation_speed());
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    azimuth_samples = config->azimuth_samples();
    encoder_size = config->encoder_size();
    config_message.header = std_msgs::msg::Header();
    config_message.header.stamp = Node::get_clock()->now();
    config_message.azimuth_samples = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->azimuth_samples()));
    config_message.encoder_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->encoder_size()));
    config_message.bin_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint64_host(config->bin_size()));
    config_message.range_in_bins = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->range_in_bins()));
    config_message.expected_rotation_rate = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->rotation_speed()));
    configuration_data_publisher->publish(config_message);

    // We only want to publish non contoured data
    radar_client.send(Type::start_non_contour_fft_data);
}