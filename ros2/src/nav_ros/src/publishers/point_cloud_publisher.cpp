#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>

#include "messages/msg/radar_configuration_message.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "point_cloud_publisher.h"
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


Point_cloud_publisher::Point_cloud_publisher():Node{ "point_cloud_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("power_threshold", 0);
    declare_parameter("azimuth_offset", 0);
    declare_parameter("combined_distance_offset", 0.0);
    declare_parameter("combined_distance_scale_factor", 0.0);
    declare_parameter("x_distance_offset", 0.0);
    declare_parameter("y_distance_offset", 0.0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    power_threshold = get_parameter("power_threshold").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();
    combined_distance_offset = get_parameter("combined_distance_offset").as_double();
    combined_distance_scale_factor = get_parameter("combined_distance_scale_factor").as_double();
    x_distance_offset = get_parameter("x_distance_offset").as_double();
    y_distance_offset = get_parameter("y_distance_offset").as_double();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Navtech::Networking::Colossus_protocol::Radar_client>(
        server_addr
    );


    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&Point_cloud_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::configuration, 
        std::bind(&Point_cloud_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<messages::msg::RadarConfigurationMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher
    );

    rclcpp::QoS qos_point_cloud_publisher(radar_point_cloud_queue_size);
    qos_point_cloud_publisher.reliable();

    point_cloud_publisher =
    Node::create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar_data/point_cloud",
        qos_point_cloud_publisher
    );
}


Point_cloud_publisher::~Point_cloud_publisher()
{
    stop();
}


void Point_cloud_publisher::start()
{
    radar_client->start();
}


void Point_cloud_publisher::stop()
{
    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);

    radar_client->send(Type::stop_fft_data);
    radar_client->stop();
}


void Point_cloud_publisher::publish_point_cloud(Message& msg)
{
    auto fft  = msg.view_as<Navtech::Networking::Colossus_protocol::FFT_data>();
    auto data = fft->to_vector();

    auto message = sensor_msgs::msg::PointCloud2();
    message.header = std_msgs::msg::Header();
    message.header.stamp = get_clock()->now();
    message.header.frame_id = "point_cloud";

    message.height = 1;
    message.width = intensity_values.size();
    const uint8_t data_type = 7;
    const uint8_t num_bytes = 4; //float32 as bytes

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

    message.fields = std::vector<sensor_msgs::msg::PointField>{x_field, y_field, z_field, intensity_field};

    message.is_bigendian = false;
    message.point_step = 4 * num_bytes;
    message.row_step = message.point_step * message.width;

    std::vector<uint8_t> data_vector;
    data_vector.reserve(intensity_values.size());
    for (unsigned i = 0; i < intensity_values.size(); i++) {

        float current_azimuth = (azimuth_values[i] * 0.9) * (M_PI / 180.0);
        float point_x = (bin_values[i] * (bin_size / 10000.0)) * cos(current_azimuth);
        float point_y = (bin_values[i] * (bin_size / 10000.0)) * sin(current_azimuth);

        auto vec = Point_cloud_publisher::floats_to_uint8_t_vector(point_x, point_y, 0, intensity_values[i]);
        data_vector.insert(data_vector.end(), vec.begin(), vec.end());
    }
    message.data = data_vector;
    message.is_dense = true;

    point_cloud_publisher->publish(message);
}


std::vector<uint8_t> Point_cloud_publisher::floats_to_uint8_t_vector(float x, float y, float z, float intensity)
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


void Point_cloud_publisher::fft_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;
    using Navtech::Protobuf::from_vector_into;

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
        for (unsigned bin_index = start_bin; bin_index < data.size(); bin_index++) {
            if ((bin_index >= start_bin) && (bin_index < end_bin)) {
                if (data[bin_index] > power_threshold) {
                    azimuth_values.push_back(adjusted_azimuth_index);
                    bin_values.push_back(bin_index);
                    intensity_values.push_back(data[bin_index]);
                }
            }
        }
    }

    if (azimuth_index < last_azimuth) {
        rotation_count++;
        rotated_once = true;
        Point_cloud_publisher::publish_point_cloud(msg);
        bin_values.clear();
        azimuth_values.clear();
        intensity_values.clear();
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

        double temp_combined_distance_offset = get_parameter("combined_distance_offset").as_double();
        if (temp_combined_distance_offset < 0.0 || temp_combined_distance_offset > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "Combined distance offset of %f is invalid, must be between %f and %f", temp_combined_distance_offset, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting combined distance offset to 0.0");
            set_parameter(rclcpp::Parameter("combined_distance_offset", 0.0));
        }
        else {
            combined_distance_offset = temp_combined_distance_offset;
        }

        double temp_combined_distance_scale_factor = get_parameter("combined_distance_scale_factor").as_double();
        if (temp_combined_distance_scale_factor < 0.0 || temp_combined_distance_scale_factor > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "Combined distance scale factor of %f is invalid, must be between %f and %f", temp_combined_distance_scale_factor, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting combined distance scale factor to 0.0");
            set_parameter(rclcpp::Parameter("combined_distance_scale_factor", 0.0));
        }
        else {
            combined_distance_offset = temp_combined_distance_scale_factor;
        }

        double temp_x_distance_offset = get_parameter("x_distance_offset").as_double();
        if (temp_x_distance_offset < 0.0 || temp_x_distance_offset > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "X distance offset of %f is invalid, must be between %f and %f", temp_x_distance_offset, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting X distance offset to 0.0");
            set_parameter(rclcpp::Parameter("x_distance_offset", 0.0));
        }
        else {
            x_distance_offset = temp_x_distance_offset;
        }

        double temp_y_distance_offset = get_parameter("y_distance_offset").as_double();
        if (temp_y_distance_offset < 0.0 || temp_y_distance_offset > 1000000.0) {
            RCLCPP_INFO(Node::get_logger(), "Y distance offset of %f is invalid, must be between %f and %f", temp_y_distance_offset, 0.0, 1000000.0);
            RCLCPP_INFO(Node::get_logger(), "Setting Y distance offset to 0.0");
            set_parameter(rclcpp::Parameter("y_distance_offset", 0.0));
        }
        else {
            y_distance_offset = temp_y_distance_offset;
        }

        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }

    if (!rotated_once) {
        return;
    }
}


void Point_cloud_publisher::configuration_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
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
    config_message.header.frame_id = "point_cloud";
    config_message.azimuth_samples = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->azimuth_samples()));
    config_message.encoder_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->encoder_size()));
    config_message.bin_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint64_host(config->bin_size()));
    config_message.range_in_bins = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->range_in_bins()));
    config_message.expected_rotation_rate = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->rotation_speed()));
    configuration_data_publisher->publish(config_message);

    RCLCPP_INFO(Node::get_logger(), "Starting point cloud publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %i", power_threshold);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);
    RCLCPP_INFO(Node::get_logger(), "Combined distance offset: %f", combined_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Combined distance scale factor: %f", combined_distance_scale_factor);
    RCLCPP_INFO(Node::get_logger(), "X distance offset: %f", x_distance_offset);
    RCLCPP_INFO(Node::get_logger(), "Y distance offset: %f", y_distance_offset);

    radar_client.send(Type::start_fft_data);
}