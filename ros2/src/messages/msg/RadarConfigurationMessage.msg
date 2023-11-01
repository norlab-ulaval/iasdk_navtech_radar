# A ROS message based on a configuration data message from a radar

# add a header message to hold message timestamp
std_msgs/Header header

# azimuth_samples (uint16) represented as a network order (uint8_t) byte array
uint8[] azimuth_samples

# encoder_size (uint16) represented as a network order (uint8_t) byte array
uint8[] encoder_size

# bin_size (double) represented as a network order (uint8_t) byte array
uint8[] bin_size

# range_in_bins (uint16) represented as a network order (uint8_t) byte array
uint8[] range_in_bins

# expected_rotation_rate (uint16) represented as a network order (uint8_t) byte array
uint8[] expected_rotation_rate