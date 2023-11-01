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

#ifndef COLOSSUS_MESSAGE_TEST_H
#define COLOSSUS_MESSAGE_TEST_H

#include <functional>
#include <string>
#include <string_view>

#include "Colossus_protocol.h"
#include "Colossus_client.h"
#include "Polled_timeout.h"
#include "configurationdata.pb.h"

namespace Navtech::Networking::Colossus_protocol {

    using namespace Time;

    // -----------------------------------------------------------------------------------------------------------------
    //
    class Test;

    class Send {
    public:
        using Send_callback = std::function<void(Message&, Test&)>;

        Send() = default;
        Send(Type msg);
        Send(Type msg, Send_callback msg_setup_fn);

        Type type() const;
        operator bool() const;

        void setup(Test& test);
        void take_down();
        void run();

    private:
        association_to<Test>  test { };

        Type          msg_type   { Type::invalid };
        Send_callback msg_setup  { };

         bool finished           { false };

        void cfg_handler(Radar_client& client, Message& msg);
    };


    // -----------------------------------------------------------------------------------------------------------------
    //
    class Listen_for {
    public:
        using Receive_callback = std::function<void(Message&, Test&)>;

        Listen_for() = default;
        Listen_for(Type msg);
        Listen_for(Type msg, Receive_callback msg_handler_fn);
        Listen_for(Type msg, Receive_callback msg_handler_fn, const Duration& run_for);

        Type type() const;
        operator bool() const;
        
        void setup(Test& test);
        void take_down();
        void run();

    private:
        association_to<Test>  test { };

        Type             msg_type    { Type::invalid };
        Receive_callback msg_handler { };
        Polled_timeout   timer       { 5_sec, Polled_timeout::Type::one_shot };
        bool             finished    { false };

        void on_receive(Radar_client& client, Message& msg);
    };

    // -----------------------------------------------------------------------------------------------------------------
    //
    class Test {
    public:
        enum Result { not_run, fail, pass, unknown };

        Test() = default;
        Test(std::string_view test_description, const Send& request);
        Test(std::string_view test_description, const Listen_for& response);
        Test(std::string_view test_description, const Send& request, const Listen_for& response);

        Type type() const;

        void run();

        Result result() const;
        void   result(Result r);
        Test&  operator=(Result r);

        std::string test_str()      const;
        std::string result_str()    const;
        std::string description()   const;

        bool operator==(const std::string& message_str) const;

        void  config(const Message& cfg_msg);
        const Configuration& config() const;
        const Colossus::Protobuf::ConfigurationData& config_protobuf() const;

        association_to<Radar_client> radar_client();
        void set_radar_client(Radar_client& cli);

    private:
        association_to<Radar_client> radar { };

        Send        send        { };
        Listen_for  listen_for  { };

        Message config_msg      { };
        Result  test_result     { not_run };
        

        std::string name                                      { "Unnamed test" };
        static constexpr std::size_t description_column_width { 50 };
    };

    
    std::string to_string(Test::Result result);

} // namespace Navtech::Networking::Test


#endif // COLOSSUS_MESSAGE_TEST_H