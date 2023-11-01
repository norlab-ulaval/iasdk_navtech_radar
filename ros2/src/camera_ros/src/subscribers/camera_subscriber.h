#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "messages/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"

class Camera_subscriber : public ::rclcpp::Node {
public:
    Camera_subscriber();

private:
    constexpr static int camera_configuration_queue_size{ 1 };
    constexpr static int camera_image_queue_size{ 25 };

    void configuration_data_callback(const messages::msg::CameraConfigurationMessage::SharedPtr data) const;
    void camera_image_callback(const sensor_msgs::msg::Image::SharedPtr data) const;

    rclcpp::Subscription<messages::msg::CameraConfigurationMessage>::SharedPtr camera_configuration_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_data_subscriber;
};