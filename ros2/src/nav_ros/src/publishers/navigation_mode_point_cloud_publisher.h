#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "messages/msg/radar_configuration_message.hpp"
#include <vector>

#include "Colossus_client.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"
#include "Peak_finder.h"
#include "pointer_types.h"

using Navtech::Networking::Colossus_protocol::Radar_client;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Unit;
using namespace Navtech::Networking::Colossus_protocol;

using Navtech::Protobuf::from_vector_into;
using namespace Colossus;
using namespace Navtech::Navigation;
using namespace Navtech::Networking;


class Navigation_mode_point_cloud_publisher : public ::rclcpp::Node
{
public:
    Navigation_mode_point_cloud_publisher();
    ~Navigation_mode_point_cloud_publisher();

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
    constexpr static int radar_point_cloud_queue_size{ 4 };

    // Owned components
    //
    Navtech::owner_of<Peak_finder> peak_finder { };
    Navtech::owner_of<Navtech::Networking::Colossus_protocol::Radar_client> radar_client { };

    // Radar client callbacks
    //
    void configuration_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg);
    void fft_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg);
    void navigation_config_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg);
    void navigation_data_handler(Radar_client& radar_client [[maybe_unused]], Message& msg);

    // Peak finder callback
    //
    void target_handler(const Navtech::Navigation::Azimuth_target& target_data);

    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };

    uint16_t start_azimuth{ 0 };
    uint16_t end_azimuth{ 0 };
    uint16_t start_bin{ 0 };
    uint16_t end_bin{ 0 };
    uint16_t azimuth_offset{ 0 };

    uint16_t bins_to_operate_on{ 0 };
    uint16_t min_bin{ 0 };
    double power_threshold{ 0 };
    uint32_t max_peaks_per_azimuth{ 0 };
    bool process_locally{ false };

    std::vector <float> azimuth_values;
    std::vector <float> bin_values;
    std::vector <float> intensity_values;

    int azimuth_samples{ 0 };
    int encoder_size{ 0 };
    float bin_size{ 0 };
    int range_in_bins{ 0 };
    int expected_rotation_rate{ 0 };
    int packet_rate{ 0 };
    float range_gain{ 0 };
    float range_offset{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };

    void publish_point_cloud();
    std::vector<uint8_t> floats_to_uint8_t_vector(float x, float y, float z, float intensity);
    void update_navigation_config();
    void update_local_navigation_config();
    void check_config_publish();

    messages::msg::RadarConfigurationMessage config_message = messages::msg::RadarConfigurationMessage{};
    Colossus_protocol::Configuration *config{};

    rclcpp::Publisher<messages::msg::RadarConfigurationMessage>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher{};
};