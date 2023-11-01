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

#ifndef COLOSSUS_CLIENT_H
#define COLOSSUS_CLIENT_H

#include "Colossus_protocol.h"
#include "Endpoint.h"
#include "Stream_client.h"
#include "Message_dispatchers.h"
#include "Time_utils.h"

namespace Navtech::Networking::Colossus_protocol {

    // Client for handling Colossus messages to/from a radar
    //
    class Radar_client {
    public:
        using Handler    = std::function<void(Radar_client&, Message&)>;
        using Dispatcher = Message_dispatcher<Protocol::colossus, Radar_client>;
       
        Radar_client(const Endpoint& server_addr);

        void start();
        void stop();

        void send(const Message& msg);
        void send(Message&& msg);

        void set_handler(Type type, const Handler& handler);
        void remove_handler(Type type);

    private:
        Stream_client<Protocol::colossus>   client;
        Dispatcher                          msg_dispatcher;
    };

} // namespace Navtech::Networking::Colossus_protocol

#endif // COLOSSUS_CLIENT_H