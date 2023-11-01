#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>

#include "messages/msg/radar_configuration_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_publisher.h"
#include "net_conversion.h"
#include "Endpoint.h"

#include "Colossus_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

using Navtech::Networking::Colossus_protocol::Radar_client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking;
using namespace Navtech::Networking::Colossus_protocol;


Laser_scan_publisher::Laser_scan_publisher():Node{ "laser_scan_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("power_threshold", 0);
    declare_parameter("azimuth_offset", 0);
    declare_parameter("range_offset", 0.0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    power_threshold = get_parameter("power_threshold").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();
    range_offset = get_parameter("range_offset").as_double();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Navtech::Networking::Colossus_protocol::Radar_client>(
        server_addr
    );

    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&Laser_scan_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::configuration, 
        std::bind(&Laser_scan_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<messages::msg::RadarConfigurationMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher
    );

    rclcpp::QoS qos_laser_scan_publisher(radar_laser_scan_queue_size);
    qos_laser_scan_publisher.reliable();

    laser_scan_publisher =
    Node::create_publisher<sensor_msgs::msg::LaserScan>(
        "radar_data/laser_scan",
        qos_laser_scan_publisher
    );
}


Laser_scan_publisher::~Laser_scan_publisher()
{
    stop();
}


void Laser_scan_publisher::start()
{
    radar_client->start();
}


void Laser_scan_publisher::stop()
{
    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);

    radar_client->send(Type::stop_fft_data);
    radar_client->stop();
}


void Laser_scan_publisher::publish_laser_scan(Message& msg)
{
    auto message = sensor_msgs::msg::LaserScan();

    auto fft  = msg.view_as<Navtech::Networking::Colossus_protocol::FFT_data>();
    auto data = fft->to_vector();

    message.header = std_msgs::msg::Header();
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "laser_frame";

    message.angle_min = M_PI / 180 * 360 / azimuth_samples * start_azimuth;
    message.angle_max = M_PI / 180 * 360 / azimuth_samples * end_azimuth;
    message.angle_increment = M_PI / 180 * 360 / azimuth_samples;
    message.time_increment = 1.0 / (expected_rotation_rate / 1000) / azimuth_samples;
    message.scan_time = 1.0 / expected_rotation_rate;
    message.range_min = bin_size / 10000.0;
    message.range_max = range_in_bins * bin_size / 10000.0;
    message.ranges.resize(azimuth_samples);
    message.ranges = range_values;
    message.intensities.resize(azimuth_samples);
    message.intensities = intensity_values;

    laser_scan_publisher->publish(message);
}


void Laser_scan_publisher::fft_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;
    using Navtech::Protobuf::from_vector_into;

    auto fft  = msg.view_as<Colossus_protocol::FFT_data>();
    auto data = fft->to_vector();

    unsigned first_peak_bin_index;
    for (first_peak_bin_index = start_bin; first_peak_bin_index < std::min((unsigned int)data.size(), (unsigned int)end_bin); first_peak_bin_index++) {
        if (data[first_peak_bin_index] > power_threshold) {
            break;
        }
    }
    
    float range = (bin_size * (first_peak_bin_index + 1) / 10000.0) + range_offset;
    float intensity = data[first_peak_bin_index];
    int azimuth_index = (int)(fft->azimuth() / (encoder_size / azimuth_samples));

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    int adjusted_azimuth_index = azimuth_index + azimuth_offset;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - azimuth_samples;
    }

    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {
        range_values[adjusted_azimuth_index] = range;
        intensity_values[adjusted_azimuth_index] = intensity;
    }
    else{
        range_values[adjusted_azimuth_index] = 0;
        intensity_values[adjusted_azimuth_index] = 0;
    }

    if (azimuth_index < last_azimuth) {
        rotation_count++;
        rotated_once = true;
        Laser_scan_publisher::publish_laser_scan(msg);
    }
    last_azimuth = azimuth_index;

    if (rotation_count >= config_publish_count) {

        int temp_azimuth_offset = get_parameter("azimuth_offset").as_int();
        if (temp_azimuth_offset > azimuth_samples) {
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

        int temp_power_threshold = get_parameter("power_threshold").as_int();
        if (temp_power_threshold < 0 || temp_power_threshold > std::numeric_limits<uint8_t>::max()) {
            RCLCPP_INFO(Node::get_logger(), "Power threshold of %i is invalid, must be between 0 and %i", temp_power_threshold, std::numeric_limits<uint8_t>::max());
            RCLCPP_INFO(Node::get_logger(), "Setting power threshold to %i", std::numeric_limits<uint8_t>::max() / 2);
            set_parameter(rclcpp::Parameter("power_threshold", power_threshold));
        }
        else {
            power_threshold = temp_power_threshold;
        }

        double temp_range_offset = get_parameter("range_offset").as_double();
        if (temp_range_offset < 0.0 || temp_range_offset > ((bin_size / 10000) * range_in_bins)) {
            RCLCPP_INFO(Node::get_logger(), "Range offset of %f is invalid, must be between %f and %f", temp_range_offset, 0.0, ((bin_size / 10000) * range_in_bins));
            RCLCPP_INFO(Node::get_logger(), "Setting range offset to 0.0");
            set_parameter(rclcpp::Parameter("range_offset", 0.0));
        }
        else {
            range_offset = temp_range_offset;
        }

        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }

    if (!rotated_once) {
        return;
    }
}


void Laser_scan_publisher::configuration_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
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
    bin_size = config->bin_size();
    end_bin = config->range_in_bins();
    range_in_bins = config->range_in_bins();
    expected_rotation_rate = config->rotation_speed();
    config_message.header = std_msgs::msg::Header();
    config_message.header.stamp = get_clock()->now();
    config_message.header.frame_id = "laser_frame";
    config_message.azimuth_samples = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->azimuth_samples()));
    config_message.encoder_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->encoder_size()));
    config_message.bin_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint64_host(config->bin_size()));
    config_message.range_in_bins = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->range_in_bins()));
    config_message.expected_rotation_rate = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->rotation_speed()));
    configuration_data_publisher->publish(config_message);

    range_values.resize(azimuth_samples);
    intensity_values.resize(azimuth_samples);

    RCLCPP_INFO(Node::get_logger(), "Starting laser scan publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %i", power_threshold);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);
    RCLCPP_INFO(Node::get_logger(), "Range offset: %f", range_offset);

    radar_client.send(Type::start_fft_data);
}