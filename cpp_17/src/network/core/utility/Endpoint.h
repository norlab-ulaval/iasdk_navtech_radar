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

#ifndef ENDPOINT_H
#define ENDPOINT_H

#include <cstdint>
#include <string>
#ifdef __linux__
#include <arpa/inet.h>
#elif _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include "IP_address.h"
#include "net_conversion.h"

namespace Navtech::Networking {

    using Port = std::uint16_t;

    struct Endpoint {
        IP_address ip_address { IP_address::any() };
        Port       port       { 0 };


        Endpoint() = default;
        
        Endpoint(Port p) : port { p }
        {
        }

        Endpoint(IP_address ip_addr, Port p) : 
            ip_address  { ip_addr },
            port        { p }
        {
        }

        Endpoint(const struct sockaddr& socket_addr, Port p)
        {
            union Address{
                struct sockaddr sa;
                struct sockaddr_in sa_ipv4;
            };

            Address addr { socket_addr };
            ip_address = IP_address { addr.sa_ipv4.sin_addr.s_addr, Endian::network };
            port       = p;
        }

        Endpoint(const struct sockaddr_in& socket_addr) :
            ip_address { socket_addr.sin_addr.s_addr, Endian::network },
            port       { to_uint16_host(socket_addr.sin_port) }
        {
        }

        std::string to_string() const
        {
            return ip_address.to_string() + ":" + std::to_string(port);
        }

        struct sockaddr_in to_sockaddr() const
        {
            sockaddr_in addr { };
            addr.sin_family         = AF_INET;
            addr.sin_addr.s_addr    = ip_address.value_as(Endian::network);
            addr.sin_port           = to_uint16_network(port);

            return addr;
        }

        struct ip_mreq to_mreq() const
        {
            struct ip_mreq mreq { };
            mreq.imr_multiaddr.s_addr = ip_address.value_as(Endian::network);
            mreq.imr_interface.s_addr = IP_address::any().value_as(Endian::network);

            return mreq;
        }

        static Endpoint null()
        {
            return Endpoint { };
        }

        operator bool() const
        {
            return port != 0;
        }
    };


    inline bool operator== (const Endpoint& lhs, const Endpoint& rhs)
    {
        return ((lhs.ip_address == rhs.ip_address) && (lhs.port == rhs.port));
    }


    inline bool operator!= (const Endpoint& lhs, const Endpoint& rhs)
    {
        return !(lhs == rhs);
    }

} // namespace Navtech::Networking

#endif // ENDPOINT_H