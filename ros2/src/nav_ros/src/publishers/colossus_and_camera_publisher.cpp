#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "messages/msg/radar_fft_data_message.hpp"
#include "messages/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "colossus_and_camera_publisher.h"
#include "net_conversion.h"
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


Colossus_and_camera_publisher::Colossus_and_camera_publisher():Node{ "colossus_and_camera_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("camera_url", "");

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    camera_url = get_parameter("camera_url").as_string();

    // Set up the radar client
    //
    Endpoint server_addr { Navtech::Networking::IP_address(radar_ip), radar_port };
    radar_client = Navtech::allocate_owned<Navtech::Networking::Colossus_protocol::Radar_client>(
        server_addr
    );

    radar_client->set_handler(
        Type::fft_data, 
        std::bind(&Colossus_and_camera_publisher::fft_data_handler, this, std::placeholders::_1, std::placeholders::_2)
    );
    radar_client->set_handler(
        Type::configuration, 
        std::bind(&Colossus_and_camera_publisher::configuration_data_handler, this, std::placeholders::_1, std::placeholders::_2)
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

    rclcpp::QoS qos_camera_configuration_publisher(camera_configuration_queue_size);
    qos_camera_configuration_publisher.reliable();

    camera_configuration_publisher =
    Node::create_publisher<messages::msg::CameraConfigurationMessage>(
        "camera_data/camera_configuration_data",
        qos_camera_configuration_publisher
    );

    rclcpp::QoS qos_camera_image_publisher(camera_image_queue_size);
    qos_camera_image_publisher.reliable();

    camera_image_publisher =
    Node::create_publisher<sensor_msgs::msg::Image>(
        "camera_data/camera_image_data",
        qos_camera_image_publisher
    );
}


Colossus_and_camera_publisher::~Colossus_and_camera_publisher()
{
    stop();
}


void Colossus_and_camera_publisher::start()
{
    radar_client->start();
}


void Colossus_and_camera_publisher::stop()
{
    radar_client->remove_handler(Type::configuration);
    radar_client->remove_handler(Type::fft_data);

    radar_client->send(Type::stop_fft_data);
    radar_client->stop();

    rclcpp::shutdown();
    RCLCPP_INFO(Node::get_logger(), "Stopped radar client");
    RCLCPP_INFO(Node::get_logger(), "Stopped camera publisher");
}


void Colossus_and_camera_publisher::fft_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;
    using namespace Navtech::Time::Monotonic;

    auto fft  = msg.view_as<Colossus_protocol::FFT_data>();
    auto data = fft->to_vector();

    auto message = messages::msg::RadarFftDataMessage();
    message.angle = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(fft->azimuth() / azimuth_samples * 360.0));
    message.azimuth = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(fft->azimuth()));
    message.sweep_counter = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(fft->sweep_counter()));
    message.ntp_seconds = Navtech::Networking::to_vector(Navtech::Networking::to_uint32_network(fft->ntp_seconds()));
    message.ntp_split_seconds = Navtech::Networking::to_vector(Navtech::Networking::to_uint32_network(fft->ntp_split_seconds()));
    message.data = data;
    message.data_length = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(data.size()));

	if (camera_message.height <= 0 || camera_message.width <= 0)
	{
        return;
	}

    // On every radar rotation, send out the latest camera frame
    if (fft->azimuth() < last_azimuth) {
        rotation_count++;
        rotated_once = true;
        camera_image_publisher->publish(camera_message);
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


void Colossus_and_camera_publisher::configuration_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg)
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
    config_message.azimuth_samples = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->azimuth_samples()));
    config_message.encoder_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->encoder_size()));
    config_message.bin_size = Navtech::Networking::to_vector(Navtech::Networking::to_uint64_host(config->bin_size()));
    config_message.range_in_bins = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->range_in_bins()));
    config_message.expected_rotation_rate = Navtech::Networking::to_vector(Navtech::Networking::to_uint16_network(config->rotation_speed()));
    fps = config->rotation_speed() / 1000;
    configuration_data_publisher->publish(config_message);

    // We only want to publish non contoured data
    radar_client.send(Type::start_non_contour_fft_data);
}


void Colossus_and_camera_publisher::camera_image_handler(cv::Mat image)
{
    auto buffer_length = image.cols * image.rows * sizeof(uint8_t) * 3;
    std::vector<uint8_t> vector_buffer(image.ptr(0), image.ptr(0) + buffer_length);

    if ((!configuration_sent) || (frame_count >= config_publish_count)) {
        auto config_message = messages::msg::CameraConfigurationMessage();
        config_message.width = image.cols;
        config_message.height = image.rows;
        config_message.channels = image.channels();
        config_message.fps = fps;
        camera_configuration_publisher->publish(config_message);
        configuration_sent = true;
        frame_count = 0;
    }

    frame_count++;

    camera_message.header = std_msgs::msg::Header();
    camera_message.header.stamp = Node::get_clock()->now();
    camera_message.header.frame_id = "camera_image";

    camera_message.height = image.rows;
    camera_message.width = image.cols;
    camera_message.encoding = "8UC3";
    camera_message.is_bigendian = true;
    camera_message.step = image.step;
    camera_message.data = std::move(vector_buffer);
}