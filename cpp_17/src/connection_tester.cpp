/// ---------------------------------------------------------------------------------------------------------------------
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

#include <vector>
#include <array>

#include "Colossus_client.h"
#include "Log.h"
#include "Time_utils.h"
#include "Colossus_protocol.h"
#include "Protobuf_helpers.h"
#include "Option_parser.h"
#include "health.pb.h"
#include "configurationdata.pb.h"
#include "Statistical_value.h"
#include "Signal_handler.h"
#include "Active.h"
#include "IP_address.h"
#include "pointer_types.h"
#include "Endpoint.h"

using Navtech::allocate_owned;
using Navtech::owner_of;

using Navtech::Networking::Colossus_protocol::Radar_client;
using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using Navtech::Utility::Option_parser;
using Navtech::Utility::Active;

using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;

using namespace Navtech::Unit;

using namespace Navtech::Networking::Colossus_protocol;
using Navtech::Networking::IP_address;
using Navtech::Networking::Port;
using Navtech::Networking::Endpoint;



// ---------------------------------------------------------------------------------------------------------------------
// A Connection_tester runs in its own thread of control and randomly connects and disconnects
// to a server.  Connection_testers can be run in parallel; to the same server, if required.
//
namespace Navtech {

    class Connection_tester : public Active {
    public:
        Connection_tester(const Endpoint& server_address);
        Connection_tester(const IP_address& server_address, Networking::Port server_port);

    protected:
        Active::Task_state run() override;
        void on_stop()           override;
        void on_start()          override;

    private:
        // owner_of<Radar_client> radar_client { };

        IP_address          ip_address { };
        Networking::Port    port       { };

        Duration min_connection_time { 500_msec };
        Duration max_connection_time { 10_sec };
        Duration min_dwell_time      { 2_sec };
        Duration max_dwell_time      { 10_sec };

        bool done { false };

        void process_config(Radar_client& radar_client, Message& msg);
        void process_FFT(Radar_client& radar_client, Message& msg);

        Duration random_duration_between(const Duration& min, const Duration& max);
    };


    Connection_tester::Connection_tester(const IP_address& server_address, Networking::Port server_port) :
        Active      { "Connection tester" },
        ip_address  { server_address },
        port        { server_port }
    {
    }


    Connection_tester::Connection_tester(const Endpoint& server_address) :
        Active      { "Connection tester" },
        ip_address  { server_address.ip_address },
        port        { server_address.port }
    {
    }


    Duration Connection_tester::random_duration_between(const Duration& min, const Duration& max)
    {
        auto min_msec = min.milliseconds();
        auto max_msec = max.milliseconds();
        auto duration = std::rand() % max_msec;

        if (duration < min_msec) duration = min_msec;
        if (duration > max_msec) duration = max_msec;

        return to_msec_duration(duration);
    }


    void Connection_tester::process_config(Radar_client& radar_client, Message& msg [[maybe_unused]])
    {
        // Ignore the config for this example, just request
        // FFT data (to load up the server).
        //
        radar_client.send(Type::start_fft_data);
    }


    void Connection_tester::process_FFT(Radar_client& radar_client [[maybe_unused]], Message& msg [[maybe_unused]])
    {
        // Consume the FFT data, without processing
    }


    Active::Task_state Connection_tester::run()
    {
        auto radar_client = allocate_owned<Radar_client>(Networking::Endpoint { ip_address, port });

        // Using a lambda expression as the handler for an incoming message.  This is the
        // same functionality as provided by the member function process_config().
        //
        radar_client->set_handler(
            Type::configuration,
            [](Radar_client& client, Message&) { client.send(Type::start_fft_data); }
        );

        // An example of using a member function as a message handler
        //
        radar_client->set_handler(
            Type::fft_data,
            std::bind(&Connection_tester::process_FFT, this, std::placeholders::_1, std::placeholders::_2)
        );
        
        auto connection_time = random_duration_between(min_connection_time, max_connection_time);
        stdout_log << "connection tester - Connecting for " << connection_time << endl;
        
        radar_client->start();
        sleep_for(connection_time);
        
        stdout_log << "connection tester - Disconnecting client" << endl;
        radar_client->remove_handler(Type::configuration);
        radar_client->remove_handler(Type::fft_data);
        radar_client->stop();
        radar_client = nullptr;

        auto dwell_time = random_duration_between(min_dwell_time, max_dwell_time);
        stdout_log << "Connection tester - Waiting for " << dwell_time << endl;
        sleep_for(dwell_time);

        if (done) return Task_state::finished;
        else      return Task_state::not_finished;
    }


    void Connection_tester::on_start()
    {
        done = false;
    }


    void Connection_tester::on_stop()
    {
        done = true;
    }

} // namespace Navtech


// ---------------------------------------------------------------------------------------------------------------------
// Signal handling: If SIGINT or SIGTERM are sent to the 
// program, stop processing.
//
volatile bool running { true };

void stop_running(std::int32_t signal [[maybe_unused]], std::int32_t info [[maybe_unused]])
{
    stdout_log << "Ctrl-C received.  Sending asynchronous stop to all connection testers..." << endl;
    running = false;
}


// ---------------------------------------------------------------------------------------------------------------------
//
Option_parser options {
    "connection_tester",
    {
        Option { "--ipaddress", "-i", "Colossus server IP address", optional, has_argument, "127.0.0.1" },
        Option { "--port", "-p",      "Colossus server port",       optional, has_argument, "6317" },
        Option { "--count", "-c",     "Max number of connections",  optional, has_argument, "1" }
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
    signal_handler.register_handler(SIGINT,  stop_running);
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
    auto connections = options["-c"].to_int();
    
    std::vector<owner_of<Navtech::Connection_tester>> connectors { };

    for (int i { 0 }; i < connections; ++i) {
        connectors.emplace_back(allocate_owned<Navtech::Connection_tester>(server_addr, server_port));
    }

    for (const auto& ct : connectors) {
        ct->start();
    }

    while (running) {
        sleep_for(500_msec);
    }

    for (const auto& ct : connectors) {
        ct->stop();
        ct->join();
    }
}
catch (std::exception& ex) {
    stdout_log << "TERMINATING MAIN DUE TO EXCEPTION: ";
    stdout_log << Logging_level::error << ex.what();
    stdout_log << endl;
}
catch (...) {
    stdout_log << "TERMINATING MAIN DUE TO UNHANDLED EXCEPTION: ";
}
