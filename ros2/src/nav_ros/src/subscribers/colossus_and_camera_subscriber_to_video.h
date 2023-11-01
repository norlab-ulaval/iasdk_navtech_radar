#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "messages/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"


class Colossus_and_camera_subscriber_to_video : public ::rclcpp::Node {
public:
    Colossus_and_camera_subscriber_to_video();

private:
    constexpr static int camera_configuration_queue_size{ 1 };
    constexpr static int camera_image_queue_size{ 25 };
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };

    bool config_data_received{ false };
    bool camera_config_data_received{ false };
    std::string output_folder_name {"output_videos"};
    cv::VideoWriter video_writer_colossus{};
    cv::VideoWriter video_writer_camera{};
    int encoder_size{ 0 };
    int azimuth_samples{ 0 };
    int video_width{ 0 };
    int video_height{ 0 };
    int azimuth{ 0 };
    int data_length{ 0 };
    int current_bearing{ 0 };
    int expected_rotation_rate{ 0 };
    cv::Mat radar_image{ cv::Size(400, 400), CV_8UC1, cv::Scalar(0, 0) };
    cv::Mat blank_image{ cv::Size(400, 400), CV_8UC1, cv::Scalar(0, 0) };
    uint8_t* image_ptr = { (uint8_t*)radar_image.data };
    int last_azimuth{ 0 };
    bool first_frame{ true };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };

    messages::msg::RadarConfigurationMessage config_message = messages::msg::RadarConfigurationMessage();

    void configuration_data_callback(const messages::msg::RadarConfigurationMessage::SharedPtr msg) const;
    void fft_data_callback(const messages::msg::RadarFftDataMessage::SharedPtr msg) const;
    void camera_configuration_data_callback(const messages::msg::CameraConfigurationMessage::SharedPtr data) const;
    void camera_image_callback(const sensor_msgs::msg::Image::SharedPtr data) const;

    rclcpp::Subscription<messages::msg::CameraConfigurationMessage>::SharedPtr camera_configuration_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_data_subscriber;
    rclcpp::Subscription<messages::msg::RadarConfigurationMessage>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<messages::msg::RadarFftDataMessage>::SharedPtr fft_data_subscriber;
};

extern std::shared_ptr<Colossus_and_camera_subscriber_to_video> node;