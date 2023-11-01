#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "messages/msg/camera_configuration_message.hpp"
#include "camera_publisher.h"

std::shared_ptr<Camera_publisher> node{};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<Camera_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting camera publisher");

    cv::VideoCapture capture{ node->get_camera_url()};

    if (!capture.isOpened()) {
        RCLCPP_INFO(node->get_logger(), "Unable to connect to camera");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "Camera connected");
        RCLCPP_INFO(node->get_logger(), "Width: %f", capture.get(cv::CAP_PROP_FRAME_WIDTH));
        RCLCPP_INFO(node->get_logger(), "Height: %f", capture.get(cv::CAP_PROP_FRAME_HEIGHT)); 
        RCLCPP_INFO(node->get_logger(), "FPS: %f", capture.get(cv::CAP_PROP_FPS));
    }

    while (rclcpp::ok()) {
        cv::Mat latest_image{ };
        auto start = std::chrono::steady_clock::now();
        auto finish = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        while (elapsed.count() < 1 / (capture.get(cv::CAP_PROP_FPS) + 1)) {
            start = std::chrono::steady_clock::now();
            capture >> latest_image;
            finish = std::chrono::steady_clock::now();
            elapsed = finish - start;
        }
        node->camera_image_handler(latest_image, capture.get(cv::CAP_PROP_FPS));
        spin_some(node);
    }

    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped camera publisher");
}
