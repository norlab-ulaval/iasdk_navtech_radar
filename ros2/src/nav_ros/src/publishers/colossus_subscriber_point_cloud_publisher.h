#include <rclcpp/rclcpp.hpp>
#include "messages/msg/radar_configuration_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "Colossus_protocol.h"
#include <vector>

using namespace Navtech::Networking::Colossus_protocol;


class Colossus_subscriber_point_cloud_publisher : public ::rclcpp::Node
{
public:
    Colossus_subscriber_point_cloud_publisher();

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };
    constexpr static int radar_point_cloud_queue_size{ 4 };

    void configuration_data_callback(const messages::msg::RadarConfigurationMessage::SharedPtr msg);
    void fft_data_callback(const messages::msg::RadarFftDataMessage::SharedPtr msg);

    bool config_received { };
    uint16_t start_azimuth{ 0 };
    uint16_t end_azimuth{ 0 };
    uint16_t start_bin{ 0 };
    uint16_t end_bin{ 0 };
    uint16_t power_threshold{ 0 };
    uint16_t azimuth_offset{ 0 };

    double combined_distance_offset{ 0.0 };
    double combined_distance_scale_factor{ 0.0 };
    double x_distance_offset{ 0.0 };
    double y_distance_offset{ 0.0 };

    std::vector <float> azimuth_values;
    std::vector <float> bin_values;
    std::vector <float> intensity_values;

    int azimuth_samples{ 0 };
    int encoder_size{ 0 };
    float bin_size{ 0 };
    int range_in_bins{ 0 };
    int expected_rotation_rate{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };

    std::vector<uint8_t> floats_to_uint8_t_vector(float x, float y, float z, float intensity);

    rclcpp::Subscription<messages::msg::RadarConfigurationMessage>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<messages::msg::RadarFftDataMessage>::SharedPtr fft_data_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher{};

    void publish_point_cloud(const messages::msg::RadarFftDataMessage::SharedPtr msg);
};