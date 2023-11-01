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

#ifndef CONNECTION_TRAITS_H
#define CONNECTION_TRAITS_H

#include <vector>
#include <cstdint>

#include "Colossus_protocol_traits.h"
#include "Cat240_protocol_traits.h"
#include "NMEA_protocol_traits.h"
#include "TLS_traits.h"
#include "TCP_socket.h"
#include "UDP_socket.h"
#include "Memory_types.h"

namespace Navtech::Networking {

    template <Protocol protocol, TLS::Policy tls_policy>
    class Connection_traits { };


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_traits<Protocol::colossus, TLS::Policy::none> {
    public:
        // Types
        //
        using Socket            = TCP_socket;
        using Message_buffer    = std::vector<std::uint8_t>;

        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::colossus>;
        using Message           = typename Protocol_traits::Message;
        using ID                = typename Protocol_traits::ID;
   
        // Constants
        //

        // Functions
        //
        static void receive_header(association_to<Socket> socket, Message& msg)
        {
            auto header_sz = Protocol_traits::header_size(msg);
            auto header    = socket->receive(header_sz);
            Protocol_traits::add_header(msg, std::move(header));
        }

        static void discard_header(association_to<Socket> socket [[maybe_unused]], Message& msg)
        {
            msg.relinquish();
        }

        static void receive_payload(association_to<Socket> socket, Message& msg)
        {
            auto payload_sz     = Protocol_traits::payload_size(msg);
            auto message_buffer = socket->receive(payload_sz);
            Protocol_traits::add_payload(msg, std::move(message_buffer));
        }
    };


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_traits<Protocol::cat240, TLS::Policy::none> {
    public:
        // Types
        //
        using Socket            = UDP_socket;
        using Message_buffer    = std::vector<std::uint8_t>;

        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::cat240>;
        using Message           = typename Protocol_traits::Message;
        using ID                = typename Protocol_traits::ID;
   
        // Constants
        //

        // Functions
        //
        static void receive_header(association_to<Socket> socket, Message& msg)
        {
            // For a UDP socket, peek the receive buffer for the
            // header data.  It must be removed in a single
            // read with the payload
            //
            auto header_sz = Protocol_traits::header_size(msg);
            auto header    = socket->receive(header_sz, Socket::Read_mode::peek).second;
            Protocol_traits::add_header(msg, std::move(header));
        }

        static void discard_header(association_to<Socket> socket, Message& msg)
        {
            // Consume, and discard, the (invalid) header from the receive buffer
            //
            socket->receive(Protocol_traits::header_size(msg));
            Protocol_traits::clear(msg);
        }

        static void receive_payload(association_to<Socket> socket, Message& msg)
        {
            // For UDP, read both the header and payload as a single
            // datagram.
            //
            auto header_sz      = Protocol_traits::header_size(msg);
            auto payload_sz     = Protocol_traits::payload_size(msg);
            auto message_buffer = socket->receive(header_sz + payload_sz).second;
            Protocol_traits::replace_data(msg, std::move(message_buffer));
        }
    };

    // -----------------------------------------------------------------------------------------------------------------
    //
    template <>
    class Connection_traits<Protocol::nmea, TLS::Policy::none> {
    public:
        // Types
        //
        using Socket            = UDP_socket;
        using Message_buffer    = std::vector<std::uint8_t>;

        using Protocol_traits   = Navtech::Networking::Protocol_traits<Protocol::nmea>;
        using Message           = typename Protocol_traits::Message;
        using ID                = typename Protocol_traits::ID;
   
        // Constants
        //
        static constexpr std::size_t   read_buffer_size { 4_kB };

        // Functions
        //
        static void receive_header(association_to<Socket> socket [[maybe_unused]], Message& msg [[maybe_unused]])
        {
           // A NMEA message is all payload; so do nothing
        }

        static void discard_header(association_to<Socket> socket [[maybe_unused]], Message& msg)
        {
            Protocol_traits::clear(msg);
        }

        static void receive_payload(association_to<Socket> socket, Message& msg)
        {
            auto message_buffer = socket->receive(read_buffer_size).second;
            Protocol_traits::replace_data(msg, std::move(message_buffer));
        }
    };


} // namespace Navtech::Networking

#endif // CONNECTION_TRAITS_H