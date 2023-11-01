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

#include <string>

#include "NMEA_client.h"
#include "Message_buffer.h"
#include "Endpoint.h"

#include "Log.h"
#include "Option_parser.h"
#include "Signal_handler.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using Navtech::Utility::Option_parser;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

using namespace Navtech::Networking;
using namespace Navtech::Networking::NMEA_protocol;

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
    "navigation_client",
    {
        Option { "--ipaddress", "-i", "Colossus server IP address", optional, has_argument, "127.0.0.1" },
        Option { "--port", "-p",      "Colossus server port",       optional, has_argument, "6317" }
    }
};


// ---------------------------------------------------------------------------------------------------------------------
//
void handle_GPRMC(Client& client [[maybe_unused]], Message& msg)
{
    stdout_log << msg.to_string() << endl;
}

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

    // Command line option parsing
    //
    options.parse(argc, argv);
    auto addr = options["-i"].translate_to<IP_address>();
    auto port = options["-p"].to_int<std::uint16_t>();
    
    Endpoint server_addr { addr, port };

    stdout_log << "Connecting to server [" << server_addr.to_string() << "]" << endl; 

    Client client { server_addr };

    client.set_handler(Type::gprmc, handle_GPRMC);
    client.set_handler(Type::pashr, [](Client&, Message&) { });
    client.set_handler(Type::gphdt, [](Client&, Message&) { });
    client.start();

    while (running) {
        sleep_for(500_msec);
    }

    client.stop();   
}
catch (std::exception& ex) {
    stdout_log << "TERMINATING MAIN DUE TO EXCEPTION: ";
    stdout_log << Logging_level::error << ex.what();
    stdout_log << endl;
}
catch (...) {
    stdout_log << "TERMINATING MAIN DUE TO UNHANDLED EXCEPTION: ";
}