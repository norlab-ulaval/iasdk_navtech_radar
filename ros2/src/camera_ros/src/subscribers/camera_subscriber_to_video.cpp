#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>

#include "camera_subscriber_to_video.h"

namespace{
    bool config_data_received { false };
    cv::VideoWriter video_writer {};
    std::string output_folder_name {"output_videos"};
}

Camera_subscriber_to_video::Camera_subscriber_to_video() :
rclcpp::Node{ "camera_subscriber_to_video" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_camera_configuration_subscriber(camera_configuration_queue_size);
    qos_camera_configuration_subscriber.reliable();

    camera_configuration_subscriber =
    Node::create_subscription<messages::msg::CameraConfigurationMessage>(
    "camera_data/camera_configuration_data",
    qos_camera_configuration_subscriber,
    std::bind(&Camera_subscriber_to_video::configuration_data_callback, this, _1));

    rclcpp::QoS qos_camera_image_subscriber(camera_image_queue_size);
    qos_camera_image_subscriber.reliable();

    camera_data_subscriber =
    Node::create_subscription<sensor_msgs::msg::Image>(
    "camera_data/camera_image_data",
    qos_camera_image_subscriber,
    std::bind(&Camera_subscriber_to_video::camera_image_callback, this, _1));
}

void Camera_subscriber_to_video::configuration_data_callback(const messages::msg::CameraConfigurationMessage::SharedPtr data) const
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

    video_writer.open(output_folder_name + "/camera_output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), data->fps, cv::Size(data->width, data->height), true);
    config_data_received = true;
}

void Camera_subscriber_to_video::camera_image_callback(const sensor_msgs::msg::Image::SharedPtr data) const
{
    if (!config_data_received) {
        RCLCPP_INFO(Node::get_logger(), "Camera configuration data not yet received");
        return;
    }

    cv::Mat camera_image = cv::Mat{ static_cast<int>(data->height), static_cast<int>(data->width), CV_8UC3, data->data.data() }.clone();
    video_writer.write(camera_image);
}