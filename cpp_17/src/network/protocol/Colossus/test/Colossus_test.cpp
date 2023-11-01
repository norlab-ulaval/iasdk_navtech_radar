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

#include <iostream>

#include "Colossus_test.h"
#include "Protobuf_helpers.h"

namespace Navtech::Networking::Colossus_protocol {

    // -----------------------------------------------------------------------------------------------------------------
    //
    std::string to_string(Type t)
    {
        switch (t) {
        case Type::keep_alive                     : return "keep_alive";
        case Type::configuration                  : return "configuration";
        case Type::configuration_request          : return "configuration_request";
        case Type::start_fft_data                 : return "start_fft_data";
        case Type::stop_fft_data                  : return "stop_fft_data";
        case Type::start_health_msgs              : return "start_health_msgs";
        case Type::stop_health_msgs               : return "stop_health_msgs";
        case Type::recalibrate_rf_health          : return "recalibrate_rf_health";
        case Type::start_tracks                   : return "start_tracks";
        case Type::stop_tracks                    : return "stop_tracks";
        case Type::transmit_on                    : return "transmit_on";
        case Type::transmit_off                   : return "transmit_off";
        case Type::fft_data                       : return "fft_data";
        case Type::high_precision_fft_data        : return "high_precision_fft_data";
        case Type::health                         : return "health";
        case Type::contour_update                 : return "contour_update";
        case Type::sector_blanking_update         : return "sector_blanking_update";
        case Type::system_restart                 : return "system_restart";
        case Type::logging_levels                 : return "logging_levels";
        case Type::logging_levels_request         : return "logging_levels_request";
        case Type::set_auto_tune                  : return "set_auto_tune";
        case Type::start_nav_data                 : return "start_nav_data";
        case Type::stop_nav_data                  : return "stop_nav_data";
        case Type::set_nav_threshold              : return "set_nav_threshold";
        case Type::navigation_data                : return "navigation_data";
        case Type::set_nav_range_offset_and_gain  : return "set_nav_range_offset_and_gain";
        case Type::calibrate_accelerometer        : return "calibrate_accelerometer";
        case Type::start_accelerometer            : return "start_accelerometer";
        case Type::stop_accelerometer             : return "stop_accelerometer";
        case Type::accelerometer_data             : return "accelerometer_data";
        case Type::start_non_contour_fft_data     : return "start_non_contour_fft_data";
        case Type::set_nav_buffer_config          : return "set_nav_buffer_config";
        case Type::set_nav_bin_operation          : return "set_nav_bin_operation";
        case Type::navigation_alarm_data          : return "navigation_alarm_data";
        case Type::nav_area_rules                 : return "nav_area_rules";
        case Type::nav_radar_reset                : return "nav_radar_reset";
        case Type::nav_radar_halt                 : return "nav_radar_halt";
        case Type::navigation_config_request      : return "navigation_config_request";
        case Type::navigation_configuration       : return "navigation_configuration";
        case Type::set_navigation_configuration   : return "set_navigation_configuration";
        case Type::nav_area_rules_request         : return "nav_area_rules_request";
        case Type::time_server_status_request     : return "time_server_status_request";
        case Type::time_server_status             : return "time_server_status";
        default                                   : return "invalid";
        };
    }


    std::string to_string(Test::Result result)
    {
        switch (result) {
        case Test::pass:    return "pass";
        case Test::fail:    return "FAIL";
        case Test::not_run: return "not run";
        case Test::unknown: return "unknown";
        default:            return "invalid";  
        };
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Send
    //
    Send::Send(Type msg) :
        msg_type { msg }
    {
    }

    Send::Send(Type msg, Send_callback msg_setup_fn) :
        msg_type    { msg },
        msg_setup   { std::move(msg_setup_fn) }
    {
    }


    Type Send::type() const
    {
        return msg_type;
    }


    Send::operator bool() const
    {
        return msg_type != Type::invalid;
    }


    void Send::setup(Test& owner)
    {
        using namespace std;

        test = associate_with(owner);

        test->radar_client()->set_handler(
            Type::configuration,
            bind(&Send::cfg_handler, this, placeholders::_1, placeholders::_2)
        );
    }


    void Send::take_down()
    {
    }


    void Send::run()
    {
        while(!finished) {
            Monotonic::sleep_for(500_msec);
        }
    }


    void Send::cfg_handler(Radar_client&, Message& msg)
    {
        test->config(msg);
        test->result(Test::pass); // <= Until failed by the msg_setup callback!

        Message request { msg_type };
            
        if (msg_setup) msg_setup(request, *test);

        test->radar_client()->send(request);
        finished = true;
    }


    // -----------------------------------------------------------------------------------------------------------------
    // Listen_for
    //
    Listen_for::Listen_for(Type msg) :
        msg_type { msg }
    {
    }

    Listen_for::Listen_for(Type msg, Receive_callback msg_handler_fn) :
        msg_type    { msg },
        msg_handler { msg_handler_fn }
    {
    }


    Listen_for::Listen_for(Type msg, Receive_callback msg_handler_fn, const Duration& run_for) :
        msg_type    { msg },
        msg_handler { msg_handler_fn },
        timer       { run_for, Polled_timeout::Type::one_shot }
    {
    }


    Type Listen_for::type() const
    {
        return msg_type;
    }


    Listen_for::operator bool() const
    {
        return msg_type != Type::invalid;
    }

    
    void Listen_for::setup(Test& owner)
    {
        using namespace std;

        test = associate_with(owner);

        test->radar_client()->set_handler(
            msg_type,
            bind(&Listen_for::on_receive, this, placeholders::_1, placeholders::_2)
        );
    }


    void Listen_for::take_down()
    {
    }


    void Listen_for::run()
    {
        timer.start();

        while (true) {
            Monotonic::sleep_for(500_msec);

            if (finished || timer.expired()) break;
        }

        timer.stop();
    }


    void Listen_for::on_receive(Radar_client&, Message& msg)
    {
        if (msg_handler) {
            msg_handler(msg, *test);
        }
        else {
            // If there's no specific handler, assume the 
            // test was to acknowledge a message was received
            //
            test->result(Test::pass);
            finished = true;
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Test
    //
    Test::Test(std::string_view test_description, const Send& request) :
        send { request },
        name { test_description }
    {
    }


    Test::Test(std::string_view test_description, const Listen_for& response) :
        listen_for { response },
        name       { test_description }
    {
    }


    Test::Test(std::string_view test_description, const Send& request, const Listen_for& response) :
        send        { request },
        listen_for  { response },
        name        { test_description }
    {
    }


    void Test::run()
    {
        test_result = unknown;

        if (send) send.setup(*this);
        if (listen_for) listen_for.setup(*this);

        radar->start();

        if (send)       send.run();
        if (listen_for) listen_for.run();

        radar->stop();
    }


    Type Test::type() const
    {
        if (listen_for) return listen_for.type();
        if (send)       return send.type();
        return Type::invalid;
    }


    Test::Result Test::result() const
    {
        return test_result;
    }


    void Test::result(Test::Result r)
    {
        test_result = r;
    }


    Test& Test::operator=(Test::Result r)
    {
        result(r);
        return *this;
    }


    bool Test::operator==(const std::string& message_str) const
    {
        if (send) {
            if (message_str == to_string(send.type())) return true;
        }

        if (listen_for) {
            if (message_str == to_string(listen_for.type())) return true;
        }

        return false;
    }


    std::string Test::result_str() const
    {
        using namespace std;

        stringstream ss { };

        auto msg_name = description();
        msg_name.resize(30, ' ');

        ss << msg_name << ": ";
        ss << to_string(test_result);

        return ss.str();
    }


    std::string Test::test_str() const
    {
        using namespace std;

        stringstream ss { };

        ss << "TEST: " << name;
        if (send) {
            ss << "Sending [" << to_string(send.type()) << "] "; 
        }

        if (listen_for) {
            ss << "Listening for [" << to_string(listen_for.type()) << "]";
        }

        return ss.str();
    }


    std::string Test::description() const
    {
        return name;
    }


    association_to<Radar_client> Test::radar_client()
    {
        return radar;
    }


    void Test::set_radar_client(Radar_client& cli)
    {
        radar = associate_with(cli);
    }


    void Test::config(const Message& cfg_msg)
    {
        config_msg = cfg_msg;
    }
    
    
    const Configuration& Test::config() const
    {
        return *config_msg.view_as<Configuration>();
    }


    const Colossus::Protobuf::ConfigurationData& Test::config_protobuf() const
    {
        using Colossus::Protobuf::ConfigurationData;
        using Navtech::Protobuf::from_vector_into;

        return from_vector_into<ConfigurationData>(config().to_vector()).value();
    }

} // namespace Navtech::Networking::Test