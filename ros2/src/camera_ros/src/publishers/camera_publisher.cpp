#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "messages/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "camera_publisher.h"

Camera_publisher::Camera_publisher():rclcpp::Node{ "camera_publisher" }
{
    declare_parameter("camera_url", "");
    camera_url = get_parameter("camera_url").as_string();

    rclcpp::QoS qos_camera_configuration_publisher(camera_configuration_queue_size);
    qos_camera_configuration_publisher.reliable();

    camera_configuration_publisher =
    Node::create_publisher<messages::msg::CameraConfigurationMessage>(
    "camera_data/camera_configuration_data",
    qos_camera_configuration_publisher);

    rclcpp::QoS qos_camera_image_publisher(camera_image_queue_size);
    qos_camera_image_publisher.reliable();

    camera_image_publisher =
    Node::create_publisher<sensor_msgs::msg::Image>(
    "camera_data/camera_image_data",
    qos_camera_image_publisher);
}

void Camera_publisher::camera_image_handler(cv::Mat image, int fps)
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

    auto message = sensor_msgs::msg::Image();
    message.header = std_msgs::msg::Header();
    message.header.stamp = node->get_clock()->now();
    message.header.frame_id = "camera_image";

    message.height = image.rows;
    message.width = image.cols;
    message.encoding = "8UC3";
    message.is_bigendian = false;
    message.step = image.step;
    message.data = std::move(vector_buffer);

    camera_image_publisher->publish(message);
}