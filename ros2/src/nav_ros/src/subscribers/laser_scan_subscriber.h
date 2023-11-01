#include <rclcpp/rclcpp.hpp>


class Laser_scan_subscriber : public ::rclcpp::Node
{
public:
    Laser_scan_subscriber();

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_laser_scan_queue_size{ 4 };

    void configuration_data_callback(const messages::msg::RadarConfigurationMessage::SharedPtr msg) const;
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

    rclcpp::Subscription<messages::msg::RadarConfigurationMessage>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber;
};