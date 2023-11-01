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

#include "Colossus_client.h"
#include "Colossus_events.h"

namespace Navtech::Networking::Colossus_protocol {

    Radar_client::Radar_client(const Endpoint& server) :
        client          { server.ip_address, server.port, Colossus_protocol::event_dispatcher },
        msg_dispatcher  { *this, Colossus_protocol::event_dispatcher }
    {
    }


    void Radar_client::start()
    {
        client.start();
        msg_dispatcher.start();
    }


    void Radar_client::stop()
    {
        msg_dispatcher.stop();
        msg_dispatcher.join();

        client.stop();
        client.join();
    }


    void Radar_client::set_handler(Type type, const Handler& handler)
    {
        msg_dispatcher.attach_to(type, handler);
    }


    void Radar_client::remove_handler(Type type)
    {
        msg_dispatcher.detach_from(type);
    }


    void Radar_client::send(const Message& msg)
    {
        client.send(msg);
    }


    void Radar_client::send(Message&& msg)
    {
        client.send(std::move(msg));
    }


} // namespace Navtech::Networking::Colossus_protocol