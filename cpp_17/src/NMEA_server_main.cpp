#include "NMEA_protocol.h"
#include "NMEA_server.h"
#include "Endpoint.h"
#include "Option_parser.h"
#include "Time_utils.h"
#include "Signal_handler.h"
#include "Log.h"

using namespace Navtech;

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using Navtech::Utility::Option_parser;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

using namespace Navtech::Networking;
using namespace Navtech::Networking::NMEA_protocol;
using namespace Navtech::Utility;


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
    "colossus_client",
    {
        Option { "--ipaddress", "-i", "Colossus server IP address", optional, has_argument, "127.0.0.1" },
        Option { "--port", "-p",      "Colossus server port",       optional, has_argument, "6317" },
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

    // Command line option parsing
    //
    options.parse(argc, argv);
    auto addr = options["-i"].translate_to<IP_address>();
    auto port = options["-p"].to_int<std::uint16_t>();

    Server server { Endpoint { addr, port } };

    // Message to send

    std::string message  { "$GPRMC,143535.300,A,5135.4604939,N,00122.9738391,W,2.337,104.604,240822,0.1,E,D*11\r\n" };

    server.start();

    while (running) {
        stdout_log << "Sending..." << endl;
        server.send(Message { message });
        sleep_for(5_sec);
    }

    server.stop();

}
catch (std::exception& ex) {
    stdout_log << "TERMINATING MAIN DUE TO EXCEPTION: ";
    stdout_log << Logging_level::error << ex.what();
    stdout_log << endl;
}
catch (...) {
    stdout_log << "TERMINATING MAIN DUE TO UNHANDLED EXCEPTION: ";
}