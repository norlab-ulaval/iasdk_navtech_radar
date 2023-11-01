#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "messages/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "../../../camera_ros/src/common/video_capture_manager.h"

#include "Colossus_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"

using Navtech::Networking::Colossus_protocol::Radar_client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol;


class Colossus_and_camera_publisher : public ::rclcpp::Node
{
public:
    Colossus_and_camera_publisher();
    ~Colossus_and_camera_publisher();

    void set_camera_url(std::string url) {
        camera_url = url;
    }

    std::string get_camera_url() {
        return camera_url;
    }

    void set_radar_ip(std::string ip) {
        radar_ip = ip;
    }

    std::string get_radar_ip() {
        return radar_ip;
    }

    void get_radar_port(uint16_t port) {
        radar_port = port;
    }

    uint16_t get_radar_port() {
        return radar_port;
    }

    void camera_image_handler(cv::Mat image);

    void start();
    void stop();

private:
    constexpr static int camera_configuration_queue_size{ 1 };
    constexpr static int camera_image_queue_size{ 25 };
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };

    // Owned components
    //
    Navtech::owner_of<Navtech::Networking::Colossus_protocol::Radar_client> radar_client { };

    // Radar client callbacks
    //
    void configuration_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg);
    void fft_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg);

    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };
    std::string camera_url = { "" };

    int bearing_count { 0 };
    int azimuth_samples { 0 };
    int fps{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };
    int rotation_count{ 0 };
    int frame_count{ 0 };
    int config_publish_count{ 4 };
    bool configuration_sent{ false };

    messages::msg::RadarConfigurationMessage config_message = messages::msg::RadarConfigurationMessage();

    rclcpp::Publisher<messages::msg::CameraConfigurationMessage>::SharedPtr camera_configuration_publisher{};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_publisher{};
    sensor_msgs::msg::Image camera_message = sensor_msgs::msg::Image();
    rclcpp::Publisher<messages::msg::RadarConfigurationMessage>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<messages::msg::RadarFftDataMessage>::SharedPtr fft_data_publisher{};
};