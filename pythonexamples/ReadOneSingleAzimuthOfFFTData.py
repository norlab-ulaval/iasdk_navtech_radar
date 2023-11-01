import socket
import matplotlib.pyplot as plt
import time
import datetime

########################################################################
# Below settings need to be changed to match your setup
########################################################################
tcp_ip = '192.168.0.1'   # This is to be the source of raw radar data. It can be a real radar or the address where 
                         # the Navtech ColossusNetrecordPlayback tool is running in playback mode
tcp_port = 6317          # This is the port that the radar is using, this generally will not need to be changed
########################################################################



########################################################################
# The below settings should not be changed
########################################################################
signature_length = 16
version_length = 1
message_type_length = 1
payload_length = 4
payload_size = 0
header_length = signature_length + version_length + message_type_length + payload_length
check_signature = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe'
check_signature = [x for x in check_signature]
config_read = False
data_read = False
encoder_size = 0
version = 0
power = []
bearing = 0
timestamp_with_nanoseconds = ''
bit_depth = 0
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
    print("--------Sent Config Request---------")
    print("")

# Send a start FFT Data Request Message - type 21
def start_fft_data():
    global radar_socket
    global version
    start_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(21,1, byteorder='big') + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(start_data_message)
    print("----------Started FFT Data----------")
    print("")

# Send a stop FFT Data Request Message - type 22
def stop_fft_data():
    global radar_socket
    global version
    stop_data_message = b'\x00\x01\x03\x03\x07\x07\x0f\x0f\x1f\x1f\x3f\x3f\x7f\x7f\xfe\xfe' + int.to_bytes(version,1, byteorder='big') + int.to_bytes(22,1, byteorder='big')  + int.to_bytes(0,4, byteorder='big')
    radar_socket.send(stop_data_message)
    print("----------Stopped FFT Data----------")
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
    global data_read
    global radar_socket
    global encoder_size
    global bearing
    global timestamp_with_nanoseconds
    global bit_depth
    global message_type
    global power

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

        elif (message_type == 31 or message_type == 30): # Message type 31 is HighRes (16Bit) FFT Data 
                                                    # and message type 30 is 8bit data from the radar
            try:
                counter = int.from_bytes(payload_data[2:4], byteorder='big')
                azimuth = int.from_bytes(payload_data[4:6], byteorder='big')
                bearing=360*(azimuth/encoder_size)
                seconds = int.from_bytes(payload_data[6:10], byteorder='little')
                split_seconds = int.from_bytes(payload_data[10:14], byteorder='little')
                if (message_type == 31):  #This is the colossus message type for 16 bit FFT data
                    print("    FFT: 16Bit (HighRes)")
                else: # If the data isn't 16 bit, but we are here in the code, we must have 8 bit data
                    print("    FFT: 8Bit (Standard)")            
                print("Counter: {}".format(counter))
                print("Azimuth: {}".format(azimuth))
                print("Bearing: {}".format(round(bearing,2)))
                print("Seconds: {}".format(seconds))
                ts = datetime.datetime.fromtimestamp(seconds).strftime('%Y-%m-%d %H:%M:%S')
                print("NanoSec: {}".format(round(split_seconds,5)))
                timestamp_with_nanoseconds = "{}.{}".format(ts, str(int(round(split_seconds/10000,0))).rjust(5,'0'))
                print("Day/Tim: " + timestamp_with_nanoseconds)
                print ("\n")
                fft_data_bytes = payload_data[14:]
                if (message_type == 31):
                    for index in range (int(len(fft_data_bytes)/2)):
                        power.append(int.from_bytes(fft_data_bytes[index:index+2],byteorder='big'))
                    data_read = True
                    bit_depth = 16
                else:
                    for index in range ((len(fft_data_bytes))):
                        power.append((int(fft_data_bytes[index])))
                    data_read = True
                    bit_depth = 8
            except:
                print("Error reading FFT message")
        elif (message_type == 1):
            print("Received keepalive from radar")
        else:
            print("Unhandled message type: {}".format(message_type))

    except:
        print("Error processing message")
########################################################################



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

# Send a message to the radar to instruct it to start sending FFT data
print("Starting FFT data")
start_fft_data()

# Try (for 5 seconds) to get an FFT message from the radar
timeout = time.time() + 5
radar_socket.settimeout(5)
try:
    print("Reading one azimuth of FFT data from radar")
    while data_read == False and time.time() < timeout:
        handle_received_message()
except:
    print("Unable to read FFT data from radar")
    exit()

# Send a message to the radar to instruct it to stop sending FFT data
print("Stopping FFT data")
stop_fft_data()

# Close down the radar socket
radar_socket.shutdown(socket.SHUT_RDWR)
radar_socket.close()
print("Closed radar connection")

# Plot the single azimuth of FFT data
if len(power) <= 0:
    print("No data points to plot")
    exit()
plt.figure("One Azimuth of {}Bit FFT Radar Data".format(bit_depth))
plt.title('{}Bit FFT Radar Data from {}° at '.format(bit_depth,  round(bearing,2)) + timestamp_with_nanoseconds)
plt.plot(power, linewidth = 0.5)
plt.ylabel('Returned power', fontsize=12)
plt.xlabel('Reporting bin', fontsize=12)
plt.tight_layout()
plt.show()

########################################################################