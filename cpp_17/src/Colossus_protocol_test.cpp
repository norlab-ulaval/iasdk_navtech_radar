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

#include "Log.h"
#include "Signal_handler.h"
#include "IP_address.h"
#include "Option_parser.h"

#include "Colossus_test.h"
#include "Colossus_test_runner.h"
#include "Colossus_test_cases.h"
#include "Colossus_client.h"

using namespace Navtech::Utility;
using namespace Navtech::Networking;
using namespace Navtech::Time;
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
//
Option_parser options {
    "colossus_protocol_tester",
    {
        Option { "--ipaddress", "-i", "Colossus server IP address", optional, has_argument, "127.0.0.1" },
        Option { "--port", "-p",      "Colossus server port",       optional, has_argument, "6317" },
        Option { "--test", "-t",      "test (by message) to run",   optional, has_argument, "all" }
    }
};


int main(int argc, char* argv[])
try
{
    // Set the log output to not display time in the
    // output.
    //
    stdout_log.time_format("");
    stdout_log.min_level(Logging_level::info);
    
    stdout_log << "Colossus Protocol Tester" << endl;

    // ---------------------------------------------------------
    //
    options.parse(argc, argv);
    auto ip_addr = options["-i"].translate_to<IP_address>();
    auto port    = options["-p"].to_int<std::uint16_t>();

    Test_runner test_runner {
        Endpoint { ip_addr, port },
        {
            Test { 
                "check for keep alive on idle client",
                Listen_for { Type::keep_alive, on_keep_alive, 30_sec } 
            },

            Test { 
                "Check configuration",
                Listen_for { Type::configuration, on_config } 
            },

            Test {
                "Update contour map", 
                Send { Type::contour_update, do_contour_update } 
            },

            Test {
                "Check contoured FFT data",
                Send       { Type::start_fft_data }, 
                Listen_for { Type::fft_data, on_contoured_fft, 5_sec } 
            },

            Test { 
                "Reset contour map",
                Send { Type::contour_update } 
            },

            Test {
                "Check FFT data with reset contour map", 
                Send        { Type::start_fft_data }, 
                Listen_for  { Type::fft_data, on_non_contoured_fft, 5_sec }
            },

            Test {
                "FFT data", 
                Send        { Type::start_fft_data }, 
                Listen_for  { Type::fft_data, on_fft, 10_sec } 
            },

            Test {
                "Non-contoured FFT data",
                Send        { Type::start_non_contour_fft_data }, 
                Listen_for  { Type::fft_data, on_non_contoured_fft, 10_sec } 
            },
            
            Test { 
                "Health",
                Send        { Type::start_health_msgs }, 
                Listen_for  { Type::health, on_health, 10_sec } 
            },

            Test {
                "Sector blanking update", 
                Send { Type::sector_blanking_update, do_sector_blanking_update } 
            },
            
            Test {
                "FFT with blanked sectors", 
                Send        { Type::start_fft_data }, 
                Listen_for  { Type::fft_data, on_blanked_fft, 5_sec } 
            },

            Test {
                "Set navigation range and gain",
                Send { Type::set_nav_range_offset_and_gain, do_set_nav_offset_and_gain } 
            },
            
            Test {
                "Check range and gain config", 
                Listen_for { Type::configuration, on_config_gain_and_offset } 
            },

            Test {
                "Set nav threshold", 
                Send { Type::set_nav_threshold, do_set_nav_threshold } 
            },

            Test { 
                "Set navigation config",
                Send { Type::set_navigation_configuration, do_set_nav_config } 
            },

            Test { 
                "Request navigation config",
                Send        { Type::navigation_config_request }, 
                Listen_for  { Type::navigation_configuration, on_navigation_config } 
            },

            Test { 
                "Navigation data",
                Send        { Type::start_nav_data }, 
                Listen_for  { Type::navigation_data, on_navigation_data, 5_sec } 
            },

            // Cannot be run on a radar without an accelerometer
            //
            // Test { 
            //     "Accelerometer data",
            //     Send        { Type::start_accelerometer }, 
            //     Listen_for  { Type::accelerometer_data, on_accelerometer_data, 5_sec } 
            // },
            
            Test { 
                "Navigation alarm data",
                Send        { Type::start_nav_data }, 
                Listen_for  { Type::navigation_alarm_data, on_nav_alarm_data, 10_sec }
            },

            Test { 
                "Set navigation area rules",
                Send { Type::nav_area_rules, do_nav_area_rules }
            },

            Test {
                "Request navigation area rules", 
                Send        { Type::nav_area_rules_request }, 
                Listen_for  { Type::nav_area_rules, on_nav_area_rules }
            },

            Test {
                "Check time sync status",
                Send        { Type::time_server_status_request },
                Listen_for  { Type::time_server_status, on_time_server_status }
            }
        }
    };


    test_runner.run(options["-t"].value());
    test_runner.display_results();
    
    //
    // ---------------------------------------------------------
}
catch (...) {
    std::cout << "Terminating on unhandled exception" << std::endl;
}