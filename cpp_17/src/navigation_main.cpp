// ---------------------------------------------------------------------------------------------------------------------
// Copyright 2023 Navtech Radar Limited
// This file is part of IASDK which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//
// Disclaimer:
// Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
// any warranty of the item whatsoever, whether express, implied, or statutory,
// including, but not limited to, any warranty of merchantability or fitness
// for a particular purpose or any warranty that the contents of the item will
// be error-free.
// In no respect shall Navtech Radar incur any liability for any damages, including,
// but limited to, direct, indirect, special, or consequential damages arising
// out of, resulting from, or any way connected to the use of the item, whether
// or not based upon warranty, contract, tort, or otherwise; whether or not
// injury was sustained by persons or property or otherwise; and whether or not
// loss was sustained from, or arose out of, the results of, the item, or any
// services that may be provided by Navtech Radar.
// ---------------------------------------------------------------------------------------------------------------------

#include <stdexcept>

#include "Log.h"
#include "Colossus_client.h"
#include "Option_parser.h"
#include "Signal_handler.h"
#include "IP_address.h"
#include "Peak_finder.h"
#include "Colossus_protocol.h"
#include "Protobuf_helpers.h"

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

using namespace Navtech::Unit;

using Navtech::Networking::Colossus_protocol::Radar_client;
using Navtech::Networking::Endpoint;
using Navtech::Networking::IP_address;
using Navtech::Networking::Port;
using namespace Navtech::Networking::Colossus_protocol;

// ---------------------------------------------------------------------------------------------------------------------
// Signal handling: If SIGINT or SIGTERM are sent to the 
// program, stop processing.
//
volatile bool running { true };

void stop_running(std::int32_t signal [[maybe_unused]], std::int32_t info [[maybe_unused]])
{
    stdout_log << "Ctrl-C received.  Terminating..." << endl;
    running = false;
}


// ---------------------------------------------------------------------------------------------------------------------
// Message handling
//
Navtech::Navigation::Peak_finder peak_finder { };


void process_FFT_data(Radar_client& radar_client [[maybe_unused]], Message& msg)
{
    using namespace Navtech::Networking;

    auto fft = msg.view_as<Colossus_protocol::FFT_data>();
    
    peak_finder.fft_data_handler(
        Navtech::Navigation::FFT_data {
            fft->azimuth(),
            fft->sweep_counter(),
            fft->ntp_seconds(),
            fft->ntp_split_seconds(),
            fft->to_vector()
        }
    );
}


// This configuration handling function extracts both the message header
// and protobuf data from the incoming message
//
void process_configuration(Radar_client& radar_client, Message& msg)
{
    using Navtech::Protobuf::from_vector_into;
    using namespace Colossus;
    using namespace Navtech::Navigation;
    using namespace Navtech::Networking;

    double        threshold             { 80.0 };             // Threshold in dB
    std::uint8_t  bins_to_operate_on    { 4 };                // Radar bins window size to search for peaks in
    std::uint16_t start_bin             { 50 };               // Start Bin
    Buffer_mode   buffer_mode           { Buffer_mode::off }; // Buffer mode should only be used with a staring radar
    std::size_t   buffer_length         { 10 };               // Buffer Length
    std::uint32_t max_peaks_per_azimuth { 10 };               // Maximum number of peaks to find in a single azimuth

    auto cfg_msg = msg.view_as<Colossus_protocol::Configuration>();
    auto protobuf = from_vector_into<Protobuf::ConfigurationData>(cfg_msg->to_vector());

    Configuration_data config { 
        cfg_msg->azimuth_samples(),
        cfg_msg->encoder_size(),
        static_cast<double>(cfg_msg->bin_size()),
        cfg_msg->range_in_bins(),
        cfg_msg->rotation_speed(),
        cfg_msg->range_gain(),
        cfg_msg->range_offset()
    };
    

    peak_finder.configure(
        config,
        protobuf.value(),
        threshold,
        bins_to_operate_on,
        start_bin,
        buffer_mode,
        buffer_length,
        max_peaks_per_azimuth
    );

    // Set up a handler, then request FFT data from the radar
    //
    radar_client.set_handler(Type::fft_data, process_FFT_data);
    radar_client.send(Type::start_fft_data);
}


// This function is not a message handler; rather, it is invoked
// by the peak finder.
//
void target_handler(const Navtech::Navigation::Azimuth_target& target_data)
{
    stdout_log  << "Target handler - "
                << "Angle ["  << target_data.angle << "] "
                << "Target count [" << target_data.targets.size() << "] "
                << endl;

    for (auto t : target_data.targets) {
        stdout_log  << "\tRange [" << t.range << "] "
                    << "Power [" << t.power << + "]"
                    << endl;
    }
}

// ---------------------------------------------------------------------------------------------------------------------
//
Option_parser options {
    "navigation_client",
    {
        Option { "--ipaddress", "-i", "Colossus server IP address", optional, has_argument, "127.0.0.1" },
        Option { "--port", "-p",      "Colossus server port",       optional, has_argument, "6317" }
    }
};


// ---------------------------------------------------------------------------------------------------------------------
//
int main(int argc, char** argv)
try
{
    // Set up signal handling for ctrl-c (SIGINT)
    // and kill (SIGTERM)
    // 
    Signal_handler signal_handler { };
    signal_handler.register_handler(SIGINT, stop_running);
    signal_handler.register_handler(SIGTERM, stop_running);

    // Set the log output up with a simple time
    // format; and don't display debug messages
    //
    stdout_log.time_format("%T");
    stdout_log.min_level(Logging_level::info);
    stdout_log << "Starting..." << endl;

    // Command line option parsing
    //
    options.parse(argc, argv);
    auto server_addr = options["-i"].translate_to<IP_address>();
    auto server_port = options["-p"].to_int<std::uint16_t>();

    // Configure the Peak Finder
    //
    peak_finder.set_target_callback(target_handler);

    // Construct a radar client and set up handlers for
    // a couple of messages.
    // Note, the radar will always send a configuration message
    // upon connection, so you should provide a handler for this
    // message.
    //
    Radar_client radar_client { Endpoint { server_addr, server_port } };
    radar_client.set_handler(Type::configuration, process_configuration);

    radar_client.start();
    
    while (running) {
        sleep_for(500_msec);
    }

    radar_client.stop();
    stdout_log << "Done" << endl;
}
catch (std::exception& ex) {
    stdout_log << "TERMINATING MAIN DUE TO EXCEPTION: ";
    stdout_log << Logging_level::error << ex.what();
    stdout_log << endl;
}