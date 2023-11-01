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

#ifndef PROTOCOL_TRAITS_H
#define PROTOCOL_TRAITS_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "IP_address.h"

namespace Navtech::Networking {

    // ----------------------------------------------------------------------------------
    // The Network_protocol enum is used to select a specific set of traits for 
    // a network protocol.
    // 
    enum class Protocol { 
        cp, 
        colossus,
        vertex,
        cat240,
        nmea
    };


    // ------------------------------------------------------------------------------------------------
    // Protocol_traits is a base template for all network message types. It
    // defines a set of common aliases and functions that should (must?) be
    // supported by all network message types.
    // The Protocol_traits class provides an abstract interface that networking
    // clients can use for creating/updating messages.  In essence, it provides
    // a compile-time mapping between the client and the message interfaces.
    // Each network message type should specialise this traits template to provide
    // their own implementations of this aliases/functions.
    //
    template <Protocol>
    class Protocol_traits {
    public:
        // -------------------------------------------------------------------------------------------
        // You must provide the following type aliases for  the protocol.
        // In general, only Message needs to be uniquely defined; the other
        // aliases are either defined in terms of Message, or are (largely)
        // generic.
        //
        // using Message           = Protocol message type; e.g. Networking::Colossus_protocol::Message
        // using Pointer           = Message*;
        // using Buffer            = std::vector<std::uint8_t>;
        // using Iterator          = std::uint8_t*;
        // using Const_iterator    = const std::uint8_t*;

        // -------------------------------------------------------------------------------------------
        // Define any protocol-specific strings (for example, the protocol name)
        // These must be static constexpr const char*
        //
        // static constexpr const char* name { "Colossus" };

        // -------------------------------------------------------------------------------------------
        // The following functions must be implemented.  These
        // are used by the message Parser to construct a message
        // from a 'raw' buffer
        //

        // template <typename... Arg_Ty>
        // static Message make(Arg_Ty&&... args);
        // static void clear(Message& msg);
        //
        // static void add_header (Message& inout_msg, const Buffer& in_buffer);
        // static void add_header (Message& inout_msg, Buffer&& in_buffer);
        //
        // static void add_payload(Message& inout_msg, const Buffer& in_buffer);
        // static void add_payload(Message& inout_msg, Buffer&& in_buffer);
        //
        // static std::size_t header_size (const Message& in_msg);
        // static std::size_t payload_size(const Message& in_msg);
        //
        // static bool is_valid(const Message& in_msg);
        //
        // Pointer dyn_alloc();
        //
        // static void add_ip_address(Message& inout_msg, const Networking::IP_address& in_addr);
        // static void add_id(Message& inout_msg, int in_id);
        //
        // static Buffer to_buffer(const Message& in_msg);
    };

} // namespace Navtech::Networking

#endif // PROTOCOL_TRAITS_H