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

#ifndef DATAGRAM_CLIENT_TRAITS_H
#define DATAGRAM_CLIENT_TRAITS_H

#include <vector>
#include <cstdint>

#include "Connection_traits.h"
#include "Time_utils.h"
#include "TCP_socket.h"

using Navtech::Time::operator""_msec;

namespace Navtech::Networking {

    template <Protocol protocol, TLS::Policy tls_policy>
    class Datagram_client_traits { };


    template <>
    class Datagram_client_traits<Protocol::cat240, TLS::Policy::none> {
    public:
        // Types
        //
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::cat240, TLS::Policy::none>;
        using Socket            = typename Connection_traits::Socket;
        using Message           = typename Connection_traits::Message;
        using ID                = typename Connection_traits::ID;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;

        // Constants
        //

        // Functions
        //
    };


    template <>
    class Datagram_client_traits<Protocol::nmea, TLS::Policy::none> {
    public:
        // Types
        //
        using Connection_traits = Navtech::Networking::Connection_traits<Protocol::nmea, TLS::Policy::none>;
        using Socket            = typename Connection_traits::Socket;
        using Message           = typename Connection_traits::Message;
        using ID                = typename Connection_traits::ID;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;

        // Constants
        //

        // Functions
        //
    };

} // namespace Navtech::Networking

#endif // DATAGRAM_CLIENT_TRAITS_H