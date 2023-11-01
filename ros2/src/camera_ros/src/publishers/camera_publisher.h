#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "sensor_msgs/msg/image.hpp"

class Camera_publisher : public ::rclcpp::Node {
public:
    Camera_publisher();

    void set_camera_url(std::string url) {
        camera_url = url;
    }

    std::string get_camera_url() {
        return camera_url;
    }

    void camera_image_handler(cv::Mat image, int fps);

private:
    constexpr static int camera_configuration_queue_size{ 1 };
    constexpr static int camera_image_queue_size{ 25 };

    std::string camera_url{ "" };
    bool configuration_sent{ false };
    int frame_count{ 0 };
    int config_publish_count{ 4 };

    rclcpp::Publisher<messages::msg::CameraConfigurationMessage>::SharedPtr camera_configuration_publisher{};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_publisher{};
};

extern std::shared_ptr<Camera_publisher> node;