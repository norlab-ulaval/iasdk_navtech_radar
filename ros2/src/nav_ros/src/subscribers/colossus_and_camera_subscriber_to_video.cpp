#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>

#include "messages/msg/radar_configuration_message.hpp"
#include "messages/msg/radar_fft_data_message.hpp"
#include "colossus_and_camera_subscriber_to_video.h"
#include "net_conversion.h"

std::shared_ptr<Colossus_and_camera_subscriber_to_video> node{};


Colossus_and_camera_subscriber_to_video::Colossus_and_camera_subscriber_to_video():Node{ "colossus_and_camera_subscriber_to_video" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber =
    Node::create_subscription<messages::msg::RadarConfigurationMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Colossus_and_camera_subscriber_to_video::configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_radar_fft_subscriber(radar_fft_queue_size);
    qos_radar_fft_subscriber.reliable();

    fft_data_subscriber =
    Node::create_subscription<messages::msg::RadarFftDataMessage>(
        "radar_data/fft_data",
        qos_radar_fft_subscriber,
        std::bind(&Colossus_and_camera_subscriber_to_video::fft_data_callback, this, _1)
    );

    rclcpp::QoS qos_camera_configuration_publisher(camera_configuration_queue_size);
    qos_camera_configuration_publisher.reliable();

    camera_configuration_subscriber =
    Node::create_subscription<messages::msg::CameraConfigurationMessage>(
        "camera_data/camera_configuration_data",
        qos_camera_configuration_publisher,
        std::bind(&Colossus_and_camera_subscriber_to_video::camera_configuration_data_callback, this, _1)
    );

    rclcpp::QoS qos_camera_image_subscriber(camera_image_queue_size);
    qos_camera_image_subscriber.reliable();

    camera_data_subscriber =
    Node::create_subscription<sensor_msgs::msg::Image>(
        "camera_data/camera_image_data",
        qos_camera_image_subscriber,
        std::bind(&Colossus_and_camera_subscriber_to_video::camera_image_callback, this, _1)
    );
}


void Colossus_and_camera_subscriber_to_video::configuration_data_callback(const messages::msg::RadarConfigurationMessage::SharedPtr msg) const
{
    if (node->config_data_received) {
        return;
    }

    RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");
    auto azimuth_samples = Navtech::Networking::from_vector_to<uint16_t>(msg->azimuth_samples);
    if (azimuth_samples.has_value()) {
        node->azimuth_samples = Navtech::Networking::to_uint16_host(azimuth_samples.value());
        node->video_height = node->azimuth_samples;
        RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", node->azimuth_samples);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Azimuth Samples");
    }

    auto encoder_size = Navtech::Networking::from_vector_to<uint16_t>(msg->encoder_size);
    if (encoder_size.has_value()) {
        node->encoder_size = Navtech::Networking::to_uint16_host(encoder_size.value());
        RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", node->encoder_size);
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
        node->video_width = Navtech::Networking::to_uint16_host(range_in_bins.value());
        RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", node->video_width);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Range In Bins");
    }

    auto expected_rotation_rate = Navtech::Networking::from_vector_to<uint16_t>(msg->expected_rotation_rate);
    if (expected_rotation_rate.has_value()) {
        node->expected_rotation_rate = Navtech::Networking::to_uint16_host(expected_rotation_rate.value());
        RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", node->expected_rotation_rate);
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Expected Rotation Rate");
    }

    if (!std::filesystem::exists(output_folder_name))
    {
        std::filesystem::create_directory(output_folder_name);
    }

    node->video_writer_colossus.open(output_folder_name + "/radar_output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), node->expected_rotation_rate / 1000, cv::Size(node->azimuth_samples, node->azimuth_samples), true);
    node->config_data_received = true;
}


void Colossus_and_camera_subscriber_to_video::fft_data_callback(const messages::msg::RadarFftDataMessage::SharedPtr msg) const
{
    if (!node->config_data_received) {
        RCLCPP_INFO(Node::get_logger(), "Radar configuration data not yet received");
        return;
    }

    auto azimuth = Navtech::Networking::from_vector_to<uint16_t>(msg->azimuth);
    if (azimuth.has_value()) {
        node->azimuth = Navtech::Networking::to_uint16_host(azimuth.value());
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Azimuth");
    }

    auto data_length = Navtech::Networking::from_vector_to<uint16_t>(msg->data_length);
    if (data_length.has_value()) {
        node->data_length = Navtech::Networking::to_uint16_host(data_length.value());
    }
    else {
        RCLCPP_INFO(Node::get_logger(), "Failed to get value for: Data Length");
    }

    node->current_bearing = (static_cast<double>(node->azimuth) / static_cast<double>(node->encoder_size)) * static_cast<double>(node->azimuth_samples);

    int max_index = std::min(static_cast<int>(node->data_length), static_cast<int>(node->azimuth_samples));
    int matrix_max_index = radar_image.rows * radar_image.cols * radar_image.channels();
    for (int i{ 0 }; i < max_index; i++) {
        int index = i * 1 + node->current_bearing * radar_image.step + 1;
        if (index < matrix_max_index) {
            image_ptr[index] = static_cast<int>(msg->data[i]);
        }
    }

    if (node->azimuth < node->last_azimuth) {
        cv::Mat recovered_lin_polar_img;
        cv::Point2f center{ static_cast<float>(radar_image.cols / 2), static_cast<float>(radar_image.rows / 2) };
        double max_radius = std::min(center.y, center.x);
        linearPolar(radar_image, recovered_lin_polar_img, center, max_radius, cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS + cv::WARP_INVERSE_MAP);
        cv::Mat normalised_image(cv::Size{ azimuth_samples, azimuth_samples }, CV_8UC1, cv::Scalar{ 0, 0 });
        normalize(recovered_lin_polar_img, normalised_image, 0, 255, cv::NORM_MINMAX);
        cv::Mat rotated_image(cv::Size{ azimuth_samples, azimuth_samples }, CV_8UC1, cv::Scalar{ 0, 0 });
        rotate(normalised_image, rotated_image, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::Mat channels[3]{ blank_image, rotated_image, blank_image };
        cv::Mat merged_data;
        merge(channels, 3, merged_data);
        node->video_writer_colossus.write(merged_data);
    }
    node->last_azimuth = node->azimuth;
}


void Colossus_and_camera_subscriber_to_video::camera_configuration_data_callback(const messages::msg::CameraConfigurationMessage::SharedPtr data) const
{
    if (config_data_received) {
        return;
    }

    RCLCPP_INFO(Node::get_logger(), "Camera Configuration received");
    RCLCPP_INFO(Node::get_logger(), "Image Width: %i", data->width);
    RCLCPP_INFO(Node::get_logger(), "Image Height: %i", data->height);
    RCLCPP_INFO(Node::get_logger(), "Image Channels: %i", data->channels);
    RCLCPP_INFO(Node::get_logger(), "Video FPS: %i", data->fps);

    if (!std::filesystem::exists(output_folder_name))
    {
        std::filesystem::create_directory(output_folder_name);
    }

    node->video_writer_camera.open(output_folder_name + "/camera_output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), data->fps, cv::Size(data->width, data->height), true);
    node->camera_config_data_received = true;
}


void Colossus_and_camera_subscriber_to_video::camera_image_callback(const sensor_msgs::msg::Image::SharedPtr data) const
{
    if (!node->camera_config_data_received) {
        RCLCPP_INFO(Node::get_logger(), "Camera configuration data not yet received");
        return;
    }

    cv::Mat camera_image = cv::Mat{ static_cast<int>(data->height), static_cast<int>(data->width), CV_8UC3, data->data.data() }.clone();
    node->video_writer_camera.write(camera_image);
}