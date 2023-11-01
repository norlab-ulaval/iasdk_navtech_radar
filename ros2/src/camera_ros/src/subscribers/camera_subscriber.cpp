#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "messages/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "camera_subscriber.h"

Camera_subscriber::Camera_subscriber():rclcpp::Node{ "camera_subscriber" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_camera_configuration_subscriber(camera_configuration_queue_size);
    qos_camera_configuration_subscriber.reliable();

    camera_configuration_subscriber =
    Node::create_subscription<messages::msg::CameraConfigurationMessage>(
    "camera_data/camera_configuration_data",
    qos_camera_configuration_subscriber,
    std::bind(&Camera_subscriber::configuration_data_callback, this, _1));

    rclcpp::QoS qos_camera_image_subscriber(camera_image_queue_size);
    qos_camera_image_subscriber.reliable();

    camera_data_subscriber =
    Node::create_subscription<sensor_msgs::msg::Image>(
    "camera_data/camera_image_data",
    qos_camera_image_subscriber,
    std::bind(&Camera_subscriber::camera_image_callback, this, _1));
}

void Camera_subscriber::configuration_data_callback(const messages::msg::CameraConfigurationMessage::SharedPtr data) const
{
    RCLCPP_INFO(Node::get_logger(), "Camera Configuration received");
    RCLCPP_INFO(Node::get_logger(), "Image Width: %i", data->width);
    RCLCPP_INFO(Node::get_logger(), "Image Height: %i", data->height);
    RCLCPP_INFO(Node::get_logger(), "Image Channels: %i", data->channels);
    RCLCPP_INFO(Node::get_logger(), "Video FPS: %i", data->fps);
}

void Camera_subscriber::camera_image_callback(const sensor_msgs::msg::Image::SharedPtr data) const
{
    RCLCPP_INFO(Node::get_logger(), "Camera Data received");
    RCLCPP_INFO(Node::get_logger(), "Image Width: %i", data->width);
    RCLCPP_INFO(Node::get_logger(), "Image Height: %i", data->height);
    RCLCPP_INFO(Node::get_logger(), "Image Encoding: %s", data->encoding.c_str());
    RCLCPP_INFO(Node::get_logger(), "Is Bigendian: %s", data->is_bigendian ? "true" : "false");
    RCLCPP_INFO(Node::get_logger(), "Image step: %i", data->step);
    RCLCPP_INFO(Node::get_logger(), "Image timestamp secs: %i", data->header.stamp.sec);
    RCLCPP_INFO(Node::get_logger(), "Image timestamp nsecs: %i", data->header.stamp.nanosec);
}