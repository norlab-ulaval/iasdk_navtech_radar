#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "messages/msg/radar_fft_data_message.hpp"
#include "b_scan_publisher.h"
#include "net_conversion.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <vector>
#include "Endpoint.h"

#include "Colossus_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"

using Navtech::Networking::Colossus_protocol::Radar_client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol;


B_scan_colossus_publisher::B_scan_colossus_publisher():Node{ "b_scan_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("azimuth_offset", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Navtech::Networking::Colossus_protocol::Radar_client>(
        server_addr
    );

    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&B_scan_colossus_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::configuration, 
        std::bind(&B_scan_colossus_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos_b_scan_image_publisher(b_scan_image_queue_size);
    qos_b_scan_image_publisher.reliable();

    b_scan_image_publisher =
        Node::create_publisher<sensor_msgs::msg::Image>(
        "radar_data/b_scan_image",
        qos_b_scan_image_publisher
    );
}


B_scan_colossus_publisher::~B_scan_colossus_publisher()
{
    stop();
}


void B_scan_colossus_publisher::start()
{
    radar_client->start();
}


void B_scan_colossus_publisher::stop()
{
    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);
    
    radar_client->send(Type::stop_fft_data);
    radar_client->stop();
}


void B_scan_colossus_publisher::image_data_handler(Message& msg) {

    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;

    auto fft  = msg.view_as<Colossus_protocol::FFT_data>();
    auto data = fft->to_vector();

    if (intensity_values.size() != (unsigned)(azimuth_samples * range_in_bins)) {
        return;
    }

    auto message = sensor_msgs::msg::Image();
    message.header = std_msgs::msg::Header();
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "b_scan_image";

    message.height = azimuth_samples;
    message.width = range_in_bins;
    message.encoding = "8UC1";
    message.is_bigendian = false;
    message.step = message.width;
    message.data = std::move(intensity_values);

    b_scan_image_publisher->publish(message);
}


void B_scan_colossus_publisher::fft_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;

    auto fft  = msg.view_as<Colossus_protocol::FFT_data>();
    auto data = fft->to_vector();

    int azimuth_index = (int)(fft->azimuth() / (encoder_size / azimuth_samples));

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    int adjusted_azimuth_index = azimuth_index + azimuth_offset;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - azimuth_samples;
    }

    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {
        for (unsigned y = 0; y < data.size(); y++) {
            int adjusted_intensity_index = (adjusted_azimuth_index * range_in_bins) + y;
            if ((y >= start_bin) && (y < end_bin)) {
                intensity_values[adjusted_intensity_index] = data[y];
            }
            else {
                intensity_values[adjusted_intensity_index] = 0;
            }
        }
    }
    else {
        for (int y = 0; y < range_in_bins; y++) {
            intensity_values[(adjusted_azimuth_index * range_in_bins) + y] = 0;
        }
    }

    if (fft->azimuth() < last_azimuth) {
        rotated_once = true;
        rotation_count++;
        B_scan_colossus_publisher::image_data_handler(msg);

        for (int x = 0; x < azimuth_samples; x++) {
            for (int y = 0; y < range_in_bins; y++) {
                intensity_values.push_back(0);
            }
        }
    }
    last_azimuth = fft->azimuth();

    if (rotation_count >= config_publish_count) {

        int temp_azimuth_offset = get_parameter("azimuth_offset").as_int();
        if (temp_azimuth_offset > azimuth_samples){
            RCLCPP_INFO(Node::get_logger(), "Azimuth offset of %i is invalid, must be less than or equal to %i", temp_azimuth_offset, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting azimuth offset to %i", azimuth_samples);
            set_parameter(rclcpp::Parameter("azimuth_offset", azimuth_samples));
        }
        else {
            azimuth_offset = temp_azimuth_offset;
        }

        int temp_start_azimuth = get_parameter("start_azimuth").as_int();
        if (temp_start_azimuth < 0 || temp_start_azimuth > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "Start azimuth of %i is invalid, must be between 0 and %i", temp_start_azimuth, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting start azimuth to %i", 0);
            set_parameter(rclcpp::Parameter("start_azimuth", 0));
        }
        else {
            start_azimuth = temp_start_azimuth;
        }

        int temp_end_azimuth = get_parameter("end_azimuth").as_int();
        if (temp_end_azimuth < 0 || temp_end_azimuth > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "End azimuth of %i is invalid, must be between 0 and %i", temp_end_azimuth, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting end azimuth to %i", azimuth_samples);
            set_parameter(rclcpp::Parameter("end_azimuth", azimuth_samples));
        }
        else {
            end_azimuth = temp_end_azimuth;
        }

        int temp_start_bin = get_parameter("start_bin").as_int();
        if (temp_start_bin < 0 || temp_start_bin > range_in_bins) {
            RCLCPP_INFO(Node::get_logger(), "Start bin of %i is invalid, must be between 0 and %i", temp_start_bin, range_in_bins);
            RCLCPP_INFO(Node::get_logger(), "Setting start bin to %i", 0);
            set_parameter(rclcpp::Parameter("start_bin", 0));
        }
        else {
            start_bin = temp_start_bin;
        }

        int temp_end_bin = get_parameter("end_bin").as_int();
        if (temp_end_bin < 0 || temp_end_bin > range_in_bins) {
            RCLCPP_INFO(Node::get_logger(), "End bin of %i is invalid, must be between 0 and %i", temp_end_bin, range_in_bins);
            RCLCPP_INFO(Node::get_logger(), "Setting end bin to %i", range_in_bins);
            set_parameter(rclcpp::Parameter("end_bin", range_in_bins));
        }
        else {
            end_bin = temp_end_bin;
        }

        rotation_count = 0;
    }

    if (!rotated_once) {
        return;
    }
}


void B_scan_colossus_publisher::configuration_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg) {
    
    auto config   = msg.view_as<Configuration>();
    encoder_size = config->encoder_size();
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", config->azimuth_samples());
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", config->encoder_size());
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %i", config->bin_size());
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", config->range_in_bins());
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", config->rotation_speed());
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    azimuth_samples = config->azimuth_samples();
    range_in_bins = config->range_in_bins();
    buffer_length = azimuth_samples * range_in_bins * sizeof(uint8_t);

    for (int x = 0; x < azimuth_samples; x++) {
        for (int y = 0; y < range_in_bins; y++) {
                intensity_values.push_back(0);
            }
        }

    RCLCPP_INFO(Node::get_logger(), "Starting b scan publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);

    radar_client.send(Type::start_fft_data);
}