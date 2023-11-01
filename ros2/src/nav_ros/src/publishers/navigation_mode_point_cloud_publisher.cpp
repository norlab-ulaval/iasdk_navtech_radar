#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>
#include <iostream>

#include "messages/msg/radar_configuration_message.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "navigation_mode_point_cloud_publisher.h"
#include "net_conversion.h"
#include "Endpoint.h"

#include "Colossus_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"
#include "Peak_finder.h"
#include "pointer_types.h"
#include "Protobuf_helpers.h"

using Navtech::Networking::Colossus_protocol::Radar_client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol;


#pragma pack(1)
class Nav_pair {
public:
    Nav_pair() = default;
    Nav_pair(std::uint32_t rng, std::uint16_t pwr)
    {
        range(rng);
        power(pwr);
    }
    std::uint32_t range() const     { return Navtech::Networking::to_uint32_host(range_val); }
    void range(std::uint32_t val)   { range_val = Navtech::Networking::to_uint32_network(val); }
    std::uint16_t power() const     { return Navtech::Networking::to_uint16_host(power_val); }
    void power(std::uint16_t val)   { power_val = Navtech::Networking::to_uint16_network(val); }
private:
    std::uint32_t range_val { };
    std::uint16_t power_val { };
};
#pragma pack()


Navigation_mode_point_cloud_publisher::Navigation_mode_point_cloud_publisher() : Node { "navigation_mode_point_cloud_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("azimuth_offset", 0);

    declare_parameter("bins_to_operate_on", 0);
    declare_parameter("min_bin", 0);
    declare_parameter("power_threshold", 0.0);
    declare_parameter("max_peaks_per_azimuth", 0);
    declare_parameter("process_locally", false);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();

    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();

    bins_to_operate_on = get_parameter("bins_to_operate_on").as_int();
    min_bin = get_parameter("min_bin").as_int();
    power_threshold = get_parameter("power_threshold").as_double();
    max_peaks_per_azimuth = get_parameter("max_peaks_per_azimuth").as_int();
    process_locally = get_parameter("process_locally").as_bool();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Navtech::Networking::Colossus_protocol::Radar_client>(
        server_addr
    );

    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&Navigation_mode_point_cloud_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::configuration, 
        std::bind(&Navigation_mode_point_cloud_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::navigation_configuration, 
        std::bind(&Navigation_mode_point_cloud_publisher::navigation_config_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::navigation_data, 
        std::bind(&Navigation_mode_point_cloud_publisher::navigation_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Set up the peak finder
    //
    peak_finder = Navtech::allocate_owned<Peak_finder>();
    peak_finder->set_target_callback(std::bind(&Navigation_mode_point_cloud_publisher::target_handler, this, std::placeholders::_1));


    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher = Node::create_publisher<messages::msg::RadarConfigurationMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher
    );

    rclcpp::QoS qos_point_cloud_publisher(radar_point_cloud_queue_size);
    qos_point_cloud_publisher.reliable();

    point_cloud_publisher = Node::create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar_data/point_cloud",
        qos_point_cloud_publisher
    );
}


Navigation_mode_point_cloud_publisher::~Navigation_mode_point_cloud_publisher()
{
    stop();
}


void Navigation_mode_point_cloud_publisher::start()
{
    radar_client->start();
}


void Navigation_mode_point_cloud_publisher::stop()
{
    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);
    radar_client->remove_handler(Type::navigation_configuration);

    radar_client->send(Type::stop_nav_data);
    radar_client->send(Type::stop_fft_data);
    radar_client->stop();
}


void Navigation_mode_point_cloud_publisher::publish_point_cloud()
{
    auto message = sensor_msgs::msg::PointCloud2();
    message.header = std_msgs::msg::Header();
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "point_cloud";

    message.height = 1;
    message.width = intensity_values.size();
    constexpr uint8_t data_type = 7;
    constexpr uint8_t num_bytes = 4; //float32 as bytes

    auto x_field = sensor_msgs::msg::PointField();
    x_field.name = "x";
    x_field.offset = 0 * num_bytes;
    x_field.datatype = data_type;
    x_field.count = intensity_values.size();

    auto y_field = sensor_msgs::msg::PointField();
    y_field.name = "y";
    y_field.offset = 1 * num_bytes;
    y_field.datatype = data_type;
    y_field.count = intensity_values.size();

    auto z_field = sensor_msgs::msg::PointField();
    z_field.name = "z";
    z_field.offset = 2 * num_bytes;
    z_field.datatype = data_type;
    z_field.count = intensity_values.size();

    auto intensity_field = sensor_msgs::msg::PointField();
    intensity_field.name = "intensity";
    intensity_field.offset = 3 * num_bytes;
    intensity_field.datatype = data_type;
    intensity_field.count = intensity_values.size();

    message.fields = std::vector<sensor_msgs::msg::PointField>{ x_field, y_field, z_field, intensity_field };

    message.is_bigendian = false;
    message.point_step = 4 * num_bytes;
    message.row_step = message.point_step * message.width;

    std::vector<uint8_t> data_vector;
    data_vector.reserve(intensity_values.size());
    for (unsigned i = 0; i < intensity_values.size(); i++) {

        float current_azimuth = (azimuth_values[i] * 0.9) * (M_PI / 180.0);
        float point_x = (bin_values[i] * (bin_size / 10000.0)) * cos(current_azimuth);
        float point_y = (bin_values[i] * (bin_size / 10000.0)) * sin(current_azimuth);

        auto vec = floats_to_uint8_t_vector(point_x, point_y, 0, intensity_values[i]);
        data_vector.insert(data_vector.end(), vec.begin(), vec.end());
    }
    message.data = std::move(data_vector);
    message.is_dense = true;

    point_cloud_publisher->publish(message);
}


std::vector<uint8_t> Navigation_mode_point_cloud_publisher::floats_to_uint8_t_vector(float x, float y, float z, float intensity)
{
    uint8_t* chars_x = reinterpret_cast<uint8_t*>(&x);
    uint8_t* chars_y = reinterpret_cast<uint8_t*>(&y);
    uint8_t* chars_z = reinterpret_cast<uint8_t*>(&z);
    uint8_t* chars_intensity = reinterpret_cast<uint8_t*>(&intensity);
    return std::vector<uint8_t>{chars_x[0], chars_x[1], chars_x[2], chars_x[3],
        chars_y[0], chars_y[1], chars_y[2], chars_y[3],
        chars_z[0], chars_z[1], chars_z[2], chars_z[3],
        chars_intensity[0], chars_intensity[1], chars_intensity[2], chars_intensity[3]};
}


void Navigation_mode_point_cloud_publisher::update_navigation_config()
{
    RCLCPP_INFO(Node::get_logger(), "Updating navigation config on radar\n");
    Colossus_protocol::Navigation_config header { };

    header.bins_to_operate_on(bins_to_operate_on);
    header.min_bin_to_operate_on(min_bin);
    header.navigation_threshold(power_threshold);
    header.max_peaks_per_azimuth(max_peaks_per_azimuth);

    Colossus_protocol::Message msg { };
    msg.type(Type::set_navigation_configuration);
    msg.append(std::move(header));

    radar_client->send(msg.relinquish());
}


void Navigation_mode_point_cloud_publisher::update_local_navigation_config()
{
    RCLCPP_INFO(Node::get_logger(), "Updating local navigation config\n");

    using Navtech::Protobuf::from_vector_into;
    using namespace Colossus;
    using namespace Navtech::Navigation;
    using namespace Navtech::Networking;

    auto protobuf = from_vector_into<Protobuf::ConfigurationData>(config->to_vector());

    Configuration_data configuration { 
        config->azimuth_samples(),
        config->encoder_size(),
        static_cast<double>(config->bin_size()),
        config->range_in_bins(),
        config->rotation_speed(),
        config->range_gain(),
        config->range_offset()
    };

    RCLCPP_INFO(Node::get_logger(), "Configuring peak finder\n");

    peak_finder->configure(
        configuration,
        protobuf.value(),
        power_threshold,
        bins_to_operate_on,
        min_bin,
        Buffer_mode::off,
        10,
        max_peaks_per_azimuth
    );

    RCLCPP_INFO(Node::get_logger(), "Configured peak finder\n");
}


void Navigation_mode_point_cloud_publisher::navigation_config_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Navigation;
    auto data = msg.view_as<Colossus_protocol::Navigation_config>();

    RCLCPP_INFO(Node::get_logger(), "Received navigation config from radar");
    RCLCPP_INFO(Node::get_logger(), "Bins to operate on: %i", data->bins_to_operate_on());
    RCLCPP_INFO(Node::get_logger(), "Min bin: %i", data->min_bin_to_operate_on());
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %f", data->navigation_threshold());
    RCLCPP_INFO(Node::get_logger(), "Max peaks per azimuth: %i", data->max_peaks_per_azimuth());
}


void Navigation_mode_point_cloud_publisher::fft_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;

    auto fft = msg.view_as<Colossus_protocol::FFT_data>();
    
    peak_finder->fft_data_handler(
        Navtech::Navigation::FFT_data {
            fft->azimuth(),
            fft->sweep_counter(),
            fft->ntp_seconds(),
            fft->ntp_split_seconds(),
            fft->to_vector()
        }
    );

}


void Navigation_mode_point_cloud_publisher::target_handler(const Navtech::Navigation::Azimuth_target& target_data)
{
    Colossus_protocol::Navigation_data header { };
    header.azimuth(target_data.azimuth);
    header.ntp_seconds(target_data.ntp_seconds);
    header.ntp_split_seconds(target_data.ntp_split_seconds);

    std::vector<std::uint8_t> peak_data;
    for (unsigned t = 0; t < target_data.targets.size(); t++) {
        // This target is it metres so needs to be multipled by 1000000 to match format of navigation data
        // This power needs to be multiplied by 10 to match format of navigation data
        auto range_data = Navtech::Networking::to_vector(Navtech::Networking::to_uint32_network((uint32_t)(target_data.targets[t].range * 1000000.0)));
        auto power_data = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network((uint16_t)(target_data.targets[t].power * 10.0)));
        copy(range_data.begin(), range_data.end(), back_inserter(peak_data));
        copy(power_data.begin(), power_data.end(), back_inserter(peak_data));
    }

    Colossus_protocol::Message msg { };
    msg.type(Type::navigation_data);
    msg.append(std::move(header));
    msg.append(std::move(peak_data));

    Colossus_protocol::Message msg_copy = msg.relinquish();
    navigation_data_handler(*radar_client, msg_copy);
}

void Navigation_mode_point_cloud_publisher::navigation_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;

    auto data = msg.view_as<Colossus_protocol::Navigation_data>();
    auto nav_pair_data = data->to_vector();
    int azimuth_index = (int)(data->azimuth() / (encoder_size / azimuth_samples));

    std::size_t nav_pair_sz   = nav_pair_data.size();
    std::size_t num_nav_pairs = nav_pair_sz / sizeof(Nav_pair);
    std::vector<Nav_pair> nav_pairs { };
    nav_pairs.resize(num_nav_pairs);
    
    std::memcpy(
    nav_pairs.data(),       // Pointer to the destination memory
    nav_pair_data.data(),   // Pointer to the source
    nav_pair_sz             // Number of bytes to copy
    );

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    int adjusted_azimuth_index = azimuth_index + azimuth_offset;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - azimuth_samples;
    }
    //RCLCPP_INFO(Node::get_logger(), "Azimuth index: %i", azimuth_index);
    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {

        for (const auto peak : nav_pairs) {
            auto target_range = peak.range();
            auto target_power = peak.power();

            // This target needs to be divided by 1000000 to get range in metres
            // This power needs to be divided by 10 to get raw power
            auto target_range_cm = target_range / 10000;
            auto raw_power = target_power / 10.0;
            auto bin_index = (int)(target_range_cm / (bin_size / 100.0));

            if ((bin_index >= start_bin) && (bin_index < end_bin)) {
                azimuth_values.push_back(adjusted_azimuth_index);
                bin_values.push_back(bin_index);
                intensity_values.push_back(raw_power);
            }
        }
    }
    if (azimuth_index < last_azimuth) {
        rotation_count++;
        rotated_once = true;
        publish_point_cloud();
        bin_values.clear();
        azimuth_values.clear();
        intensity_values.clear();
    }
    last_azimuth = azimuth_index;

    check_config_publish();

    if (!rotated_once) {
        return;
    }
}


void Navigation_mode_point_cloud_publisher::check_config_publish()
{
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

        // These params mean a new config has to be sent to the radar
        bool update_radar_navigation_config = false;
        if (get_parameter("bins_to_operate_on").as_int() != bins_to_operate_on)
        {
            int temp_bins_to_operate_on = get_parameter("bins_to_operate_on").as_int();
            if (temp_bins_to_operate_on < 0 || temp_bins_to_operate_on > 10) {
                RCLCPP_INFO(Node::get_logger(), "Bins to operate on of %i is invalid, must be between 0 and %i", temp_bins_to_operate_on, 10);
                RCLCPP_INFO(Node::get_logger(), "Setting bins to operate on to %i", 10);
                set_parameter(rclcpp::Parameter("bins_to_operate_on", bins_to_operate_on));
            }
            else {
                bins_to_operate_on = temp_bins_to_operate_on;
            }
            update_radar_navigation_config = true;
        }
        if (get_parameter("min_bin").as_int() != min_bin)
        {
            int temp_min_bin = get_parameter("min_bin").as_int();
            if (temp_min_bin < 0 || temp_min_bin > range_in_bins) {
                RCLCPP_INFO(Node::get_logger(), "Min bin of %i is invalid, must be between 0 and %i", temp_min_bin, range_in_bins);
                RCLCPP_INFO(Node::get_logger(), "Setting min bin to %i", range_in_bins);
                set_parameter(rclcpp::Parameter("min_bin", min_bin));
            }
            else {
                min_bin = temp_min_bin;
            }
            update_radar_navigation_config = true;
        }

        if (get_parameter("power_threshold").as_double() != power_threshold)
        {
            double temp_power_threshold = get_parameter("power_threshold").as_double();
            if (temp_power_threshold < 0 || temp_power_threshold > std::numeric_limits<uint8_t>::max()) {
                RCLCPP_INFO(Node::get_logger(), "Power threshold of %f is invalid, must be between 0 and %f", temp_power_threshold, static_cast<double>(std::numeric_limits<uint8_t>::max()));
                RCLCPP_INFO(Node::get_logger(), "Setting power threshold to %f", static_cast<double>(std::numeric_limits<uint8_t>::max() / 4));
                set_parameter(rclcpp::Parameter("power_threshold", static_cast<double>(std::numeric_limits<uint8_t>::max() / 4)));
            }
            else {
                power_threshold = temp_power_threshold;
            }
            update_radar_navigation_config = true;
        }
        if (get_parameter("max_peaks_per_azimuth").as_int() != max_peaks_per_azimuth)
        {
            int temp_max_peaks_per_azimuth = get_parameter("max_peaks_per_azimuth").as_int();
            if (temp_max_peaks_per_azimuth < 0 || temp_max_peaks_per_azimuth > 5) {
                RCLCPP_INFO(Node::get_logger(), "Max peaks per azimuth of %i is invalid, must be between 0 and %i", temp_max_peaks_per_azimuth, 5);
                RCLCPP_INFO(Node::get_logger(), "Setting max peaks per azimuth to %i", 5);
                set_parameter(rclcpp::Parameter("max_peaks_per_azimuth", 5));
            }
            else {
                max_peaks_per_azimuth = temp_max_peaks_per_azimuth;
            }
            update_radar_navigation_config = true;
        }

        configuration_data_publisher->publish(config_message);
        rotation_count = 0;

        if (update_radar_navigation_config) {

            if (process_locally) {
                update_local_navigation_config();
            }
            else {
                update_navigation_config();
                radar_client->send(Type::navigation_config_request);
            }
        }
    }
}


void Navigation_mode_point_cloud_publisher::configuration_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    config   = msg.view_as<Colossus_protocol::Configuration>();
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
    packet_rate = config->packet_rate();
    range_gain = config->range_gain();
    range_offset = config->range_offset();
    config_message.header = std_msgs::msg::Header();
    config_message.header.stamp = get_clock()->now();
    config_message.header.frame_id = "point_cloud";
    config_message.azimuth_samples = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->azimuth_samples()));
    config_message.encoder_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->encoder_size()));
    config_message.bin_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint64_host(config->bin_size()));
    config_message.range_in_bins = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->range_in_bins()));
    config_message.expected_rotation_rate = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->rotation_speed()));
    configuration_data_publisher->publish(config_message);

    RCLCPP_INFO(Node::get_logger(), "Starting navigation mode point cloud publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);
    RCLCPP_INFO(Node::get_logger(), "Processing locally: %s", process_locally ? "true" : "false");

    if (process_locally) {
        RCLCPP_INFO(Node::get_logger(), "Processing navigation data locally");
        update_local_navigation_config();
        radar_client.send(Type::start_non_contour_fft_data); // peak_finding does not currently work with contoured data
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Processing navigation data on radar");
        update_navigation_config();
        radar_client.send(Type::start_nav_data);
    }
}