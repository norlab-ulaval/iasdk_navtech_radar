# A ROS message based on an FFT data message from a radar

# add a header message to hold message timestamp
std_msgs/Header header

# angle (double) represented as a network order (uint8_t) byte array
uint8[] angle

# azimuth (uint16_t) represented as a network order (uint8_t) byte array
uint8[] azimuth

# sweep_counter (uint16_t) represented as a network order (uint8_t) byte array
uint8[] sweep_counter

# ntp_seconds (uint32_t) represented as a network order (uint8_t) byte array
uint8[] ntp_seconds

# ntp_split_seconds (uint32_t) represented as a network order (uint8_t) byte array
uint8[] ntp_split_seconds

# data (uint8_t) represented as a network order (uint8_t) byte array
uint8[] data

# data_length (uint16_t) represented as a network order (uint8_t) byte array
uint8[] data_length