import matplotlib.cm as cm       # colourmap
import matplotlib.pyplot as plt
import socket
import struct
import numpy as np
import time
import datetime
import os

#########################################################################
### Note that this script requires radar firmware 3.0.0.131 or higher ###
#########################################################################



########################################################################
# The below settings need to be changed to match your setup
########################################################################
tcp_ip = '192.168.0.1'   # this is to be the source of raw radar data from a physical radar. Raw recorded 
                         # radar datasets played back through the Navtech playback tool cannot be used
tcp_port = 6317          # this is the port that the radar is using, this generally will not need to be changed
########################################################################



########################################################################
# The below settings can be changed to alter navigation mode behaviour
########################################################################
max_range_of_interest = 200     # in metres: this value is used to discard points beyond a certain range
                                # usually, a customer will set the radar's operating range through the radar's web ui

bins = 5                        # number of bins to operate on (window size) for peakfinding

min_bins = 3                    # closest bin to consider within the peak finding.
                                # this allows for close-in radar returns to be ignored

nav_threshold = 55              # power threshold used to configure the operation of the 
                                # navigationmode. note that setting this value too low can 
                                # result in the processing effort required from the radar
                                # exceeding that which is available. this will result in
                                # points appearing to be "missing" from ranges of azimuths
                                # in the returned dataset

max_peaks = 10                  # the number of peaks per azimuth that navigationmode should report
########################################################################



########################################################################
# The below settings should not be changed
########################################################################
signature_length = 16
version_length = 1
message_type_length = 1
payload_length = 4
header_length = signature_length + version_length + message_type_length + payload_length
check_signature = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe'
check_signature = [x for x in check_signature]
config_read = False
version = 0
message_type = 0
payload_size = 0
azimuth = 0
encoder_size = 0
packet_rate = 0
azimuth_samples = 0
first_run = True
range_list = []
bearing_list = []
power_list = []
nav_message_count = 0
nav_config_read = False
timestamp_seconds_and_fractional_seconds = 0.0
first_timestamp_seconds_and_fractional_seconds = 0.0
first_message = True
rotation_speed = 0
starting_azimuth = 0
state = "READING_HEADER"
signature_data = []
header_data = []
payload_data = []
########################################################################



########################################################################
# The below section contains function definitions
########################################################################

# Send a Configuration Request Message - type 20
def send_config_request():
    global radar_socket
    global version
    config_request_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(20,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(config_request_message)
    print("----------- Requested config -----------")
    print("")

# Send a start navigation data request message - type 120
def start_nav_data():
    global radar_socket
    global version
    start_nav_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(120,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(start_nav_message)
    print("---------- Requested nav data ----------")
    print("")

# Send a stop navigation data request message - type 121
def stop_nav_data():
    global radar_socket
    global version
    stop_nav_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(121,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(stop_nav_message)
    print("----------- Stopped nav data -----------")
    print("")

# Send a set navigation threshold message - type 122 - message type not required for radars with firmware 3.0.0.131 or later.
#def set_nav_threshold(threshold_db):
#    global radar_socket
#    global version
#    set_nav_threshold_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(122,1, byteorder='big')  + int.to_bytes(2,4, byteorder='big')+ int.to_bytes(int(thresholddb*10),2, byteorder='big')
#    radar_socket.send(set_nav_threshold_message)
#    print("----------set nav threshold {}----------".format(threshold_db))
#    print("")

# Send a set navigation configuration message - type 205
def set_nav_config(bins_to_operate_on, minimum_bin, navigation_threshold, max_peaks_per_azimuth):
    global radar_socket
    global version
    set_nav_config_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(205,1, byteorder='big')  + int.to_bytes(12,4, byteorder='big')+ int.to_bytes(int(bins_to_operate_on),2, byteorder='big')+ int.to_bytes(int(minimum_bin),2, byteorder='big')+ struct.pack('>f',navigation_threshold*10) + int.to_bytes(int(max_peaks_per_azimuth),4, byteorder='big')
    radar_socket.send(set_nav_config_message)
    print("----------- Set nav config -----------")
    print("Bins      {}".format(bins_to_operate_on))
    print("Minbin    {}".format(minimum_bin))
    print("Threshold {}".format(navigation_threshold))
    print("Maxpeaks  {}".format(max_peaks_per_azimuth))
    print("")

# Send a get navigation configuration message - type 203
def get_nav_config():
    global radar_socket
    global version
    get_nav_config_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(203,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(get_nav_config_message)
    print("-------- Requested nav config ----------")
    print("")


# Read the response message
def handle_received_message():
    global signature_length
    global header_length
    global check_signature
    global radar_socket
    global version
    global state
    global signature_data
    global header_data
    global payload_data

    if state == "READING_HEADER":
        try:
            signature_data = list(radar_socket.recv(signature_length))
            if (signature_data == check_signature):
                header_data = list(radar_socket.recv(header_length - signature_length))
                version = header_data[0]
                state = "READING_PAYLOAD"
            else:
                state = "READING_HEADER"
            return
        except:
            print("Error reading message header")
    elif state == "READING_PAYLOAD":
        try:
            payload_size = int.from_bytes(header_data[2:6], byteorder='big')
            if payload_size == 0:
                state = "DISPATCHING"
            else:
                payload_data = list(radar_socket.recv(payload_size))
                state = "DISPATCHING"
            return    
        except:
            print("Error reading message payload")
    elif state == "DISPATCHING":
        try:
            process_received_message(header_data, payload_data)  
            state = "READING_HEADER"
            return
        except:
            print("Error dispatching message")
            state = "READING_HEADER"
            return
    else:
        print("Unknow state: {}".format(state))
        return

# Process the response message
def process_received_message(header_data, payload_data):
    global config_read
    global nav_config_read
    global version
    global encoder_size
    global azimuth_samples
    global nav_message_count
    global timestamp_seconds_and_fractional_seconds
    global first_timestamp_seconds_and_fractional_seconds
    global first_message
    global rotation_speed
    global range_resolution
    global starting_azimuth
    global azimuth

    try:
        # Read message type
        message_type = header_data[1]

        # Read the rest of the message
        if (message_type == 10): # Configuration data from radar
            try:
                azimuth_samples = int.from_bytes(payload_data[0:2], byteorder='big')
                range_resolution = int.from_bytes(payload_data[2:4], byteorder='big')
                range_bins = int.from_bytes(payload_data[4:6], byteorder='big')
                encoder_size = int.from_bytes(payload_data[6:8], byteorder='big')
                rotation_speed = int.from_bytes(payload_data[8:10], byteorder='big')
                packet_rate = int.from_bytes(payload_data[10:12], byteorder='big')
                print("----------Config Message----------\n")
                print("Azimuth Samples:  {} samples/rotation".format(azimuth_samples))
                print("Range Resolution: {} mm/bin".format(range_resolution/10))
                print("Range:            {} bins".format(range_bins))
                print("Encoder Size:     {} counts/rotation".format(encoder_size))
                print("Rotation Speed:   {} mHz".format(rotation_speed))
                print("Packet Rate:      {} azimuths/second".format(packet_rate))
                config_read = True
                print("")
            except:
                print("Error reading config message")

        elif (message_type == 204): # Navigation configuration data from radar
            try:
                bins = int.from_bytes(payload_data[0:2], byteorder='big')
                min_bin = int.from_bytes(payload_data[2:4], byteorder='big')
                [threshold] = struct.unpack('>f',(bytearray(payload_data[4:8])))
                max_peaks = int.from_bytes(payload_data[8:12], byteorder='big')
                print("-- Get navigation config (message204) --")
                print("Bins      {}".format(bins))
                print("Minbin    {}".format(min_bin))
                print("Threshold {}".format(threshold/10))
                print("Maxpeaks  {}".format(max_peaks)) 
                print("") 
                nav_config_read = True
            except:
                print("Error reading navigation configuration message")

        elif (message_type == 123): # navigation data from the radar
            try:
                azimuth = int.from_bytes(payload_data[0:2], byteorder='big')
                bearing = 360*(azimuth/encoder_size)
                seconds = int.from_bytes(payload_data[2:6], byteorder='big')
                split_seconds = int.from_bytes(payload_data[6:10], byteorder='big')
                nav_data = list(payload_data[10:])
                timestamp_seconds_and_fractional_seconds = seconds + (split_seconds/1000000000)
                for index in range (int(len(nav_data)/6)):
                    byte_offset = index * 6
                    peak_range = int.from_bytes(nav_data[0+byte_offset:4+byte_offset],byteorder='big')
                    peak_power = int.from_bytes(nav_data[4+byte_offset:6+byte_offset],byteorder='big')
                    peak_range_metres = peak_range/1000000
                    peak_power_db = peak_power/10
                    if (peak_range_metres<=max_range_of_interest)and(peak_range_metres>=min_bins*range_resolution/10000):
    
                        range_list.append(peak_range_metres)
                        bearing_list.append((bearing/360)*2 * np.pi)
                        power_list.append(peak_power_db) 
                if first_message == True:
                    starting_azimuth = int(azimuth / encoder_size * azimuth_samples)
                    first_timestamp_seconds_and_fractional_seconds = timestamp_seconds_and_fractional_seconds
                    first_message = False
                nav_message_count += 1                       
            except:
                print("Error reading nav message")
        elif (message_type == 1):
            print("Received keepalive from radar")
        else:
            print("Unhandled message type: {}".format(message_type))

    except:
        print("Error processing message")
######################################################################################



########################################################################
# The below section contains the main program code
########################################################################

# Try (for 5 seconds) to connect to the radar
try:
    print("Connecting to radar at {} on port {}".format(tcp_ip, tcp_port))
    radar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    radar_socket.settimeout(5)
    radar_socket.connect((tcp_ip, tcp_port))
except:
    print("Unable to connect to radar at {} on port {}".format(tcp_ip, tcp_port))
    exit()

timestamp=datetime.datetime.now()
file_path=(timestamp.strftime('radardata/%y/%m/%d/'))	# the folder path that contains the log file
if (not os.path.exists(file_path)):	# if the folder isn't there
	os.makedirs (file_path)			# ... then create it
file_name=(file_path+(timestamp.strftime('navigationmodedata_%H-%M-%S.csv'))) # filename is made up of fixed string and day-number in month 

# Try (for 5 seconds) to get a configuration message from the radar
timeout = time.time() + 5
radar_socket.settimeout(5)
try:
    print("Reading configuration message from radar")
    while config_read == False and time.time() < timeout:
        handle_received_message()
except:
    print("Unable to read configuration message from radar")
    exit()

# Send the required navigation mode configuration to the radar
print("Setting navigation mode config")
set_nav_config(bins, min_bins, nav_threshold, max_peaks)

# Try (for 5 seconds) to get the current navigation mode settings from the radar
timeout = time.time() + 5
radar_socket.settimeout(5)
try:
    print("Reading navigation mode configuration message from radar")
    while nav_config_read == False and time.time() < timeout:
        get_nav_config()           # send a request navigation configuration message to the radar
        handle_received_message()
except:
    print("Unable to read navigation mode configuration message from radar")
    exit()


 # Instruct the radar to start sending navigation mode messages
print("Starting navigation data")
start_nav_data()

# Try (for 10 seconds) to get an entire rotation of navigation data messages from the radar
timeout = time.time() + 10
radar_socket.settimeout(10)
try:
    print("Reading one rotation of navigation data from radar")
    handle_received_message()             # Read the first nav data
    nav_data_captured = 1
    timeout = time.time() + 5
    while (int(azimuth / encoder_size * azimuth_samples) != starting_azimuth or nav_data_captured < azimuth_samples) and time.time() < timeout:
        handle_received_message()             # Read the rest of the nav data to make a complete rotation
        nav_data_captured += 1
except:
    print("Unable to read nav data from radar")
    exit()

 # Instruct the radar to stop sending navigation mode messages
print("Stopping navigation data")
stop_nav_data()

# Close down the radar socket
radar_socket.shutdown(socket.SHUT_RDWR)
radar_socket.close()
print("Closed radar connection")

# Save peaks data to file
target = open(file_name,'w')
target.write("date,{}\n".format(timestamp.strftime('%y/%m/%d')))
target.write("time,{}\n".format(timestamp.strftime('%H:%M:%S')))
target.write("threshold,{}\n".format(nav_threshold))
target.write("binstooperateon,{}\n".format(bins))
target.write("minbin,{}\n".format(min_bins))
target.write("peakstoreturn,{}\n".format(max_peaks))
target.write("notes\n")
target.write("range(m),bearing(rad),power\n")
for i in range (len(power_list)):
    target.write("{},{},{}\n".format(range_list[i],bearing_list[i],power_list[i]))
target.close()

print ("Peaks indentified: {}".format(len(power_list)))
if len(power_list) <= 0:
    print("No data points to plot")
    exit()
fig = plt.figure("One rotation of navigation mode data")
fig.set_size_inches(9.,10.)
ax = fig.add_subplot(projection='polar')
ax.set_ylim(0,max_range_of_interest)
c = ax.scatter(bearing_list, range_list, s=18, c=power_list, cmap=cm.rainbow,vmin=min(power_list), vmax=max(power_list))
ax.set_title('Navigation mode data. binstooperateon={}, minbin={}, threshold={}, maxpeaks={}'.format(bins, min_bins, nav_threshold, max_peaks))
ax.set_theta_direction(-1)
ax.set_theta_offset(np.pi / 2.0)
plt.tight_layout()
plt.savefig(file_path + (timestamp.strftime('navigationmodedata_%H-%M-%S.pdf')), dpi=2000)
plt.show()
########################################################################