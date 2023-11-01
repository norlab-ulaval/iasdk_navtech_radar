#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "messages/msg/radar_configuration_message.hpp"
#include <vector>

#include "Colossus_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

using Navtech::Networking::Colossus_protocol::Radar_client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol;


class Laser_scan_publisher : public ::rclcpp::Node
{
public:
    Laser_scan_publisher();
    ~Laser_scan_publisher();

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

    void start();
    void stop();

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_laser_scan_queue_size{ 4 };

    // Owned components
    //
    Navtech::owner_of<Navtech::Networking::Colossus_protocol::Radar_client> radar_client { };

    // Radar client callbacks
    //
    void configuration_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg);
    void fft_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg);

    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };
    uint16_t start_azimuth{ 0 };
    uint16_t end_azimuth{ 0 };
    uint16_t start_bin{ 0 };
    uint16_t end_bin{ 0 };
    uint16_t power_threshold{ 0 };
    uint16_t azimuth_offset{ 0 };
    double range_offset{ 0.0 };

    std::vector <float> range_values;
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

    messages::msg::RadarConfigurationMessage config_message = messages::msg::RadarConfigurationMessage{};

    rclcpp::Publisher<messages::msg::RadarConfigurationMessage>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher{};

    void publish_laser_scan(Message& msg);
};