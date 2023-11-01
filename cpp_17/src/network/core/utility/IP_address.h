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

#ifndef IP_ADDRESS_H
#define IP_ADDRESS_H

#include <string>
#include <string_view>
#include <cstdint>
#include <array>

#ifdef __linux__
#include <arpa/inet.h>
#elif _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif


namespace Navtech::Networking {

    enum class Endian { host, network };

    using Byte_array = std::array<std::uint8_t, 4>;

    class IP_address {
    public:
        IP_address() = default;
        IP_address(std::string_view add_str);
        IP_address(std::uint32_t raw_addr);
        IP_address(std::uint32_t raw_addr, Endian endian);
        IP_address(const sockaddr& socket_addr);
        IP_address(const sockaddr_in& socket_addr);

        IP_address& operator=(std::uint32_t ip_addr);
        IP_address& operator=(std::string_view ip_addr_str);
    
        operator std::uint32_t() const;        // host-endian
        std::string   to_string()  const;
        std::uint32_t to_host_endian() const;
        std::uint32_t to_network_endian() const;
        std::uint32_t value_as(Endian endian) const;
        Byte_array    to_byte_array() const;

        IP_address  operator~() const;

        IP_address  operator& (const IP_address& rhs) const;
        IP_address& operator&=(const IP_address& rhs);

        IP_address  operator& (std::uint32_t rhs) const;
        IP_address& operator&=(std::uint32_t rhs);
        friend IP_address operator&(std::uint32_t lhs, const IP_address& rhs);

        IP_address  operator| (const IP_address& rhs) const;
        IP_address& operator|=(const IP_address& rhs);

        IP_address  operator| (std::uint32_t rhs) const;
        IP_address& operator|=(std::uint32_t rhs);
        friend IP_address operator|(std::uint32_t lhs, const IP_address& rhs);

        bool operator==(const IP_address& rhs) const;
        bool operator==(std::uint32_t rhs) const;
        bool operator!=(const IP_address& rhs) const;
        bool operator!=(std::uint32_t rhs) const;

        // Special-case IP addresses
        //
        static IP_address localhost();  // 127.0.0.1
        static IP_address any();        // 0.0.0.0
        static IP_address null();       // 0.0.0.0

    private:
        void parse_string(std::string_view ip_addr_str);
        void validate(std::string_view ip_addr_str);

        union Address {
            std::uint32_t word;
            std::uint8_t  byte[4];
        };

        Address address { };
    };


    inline IP_address operator""_ipv4(const char* addr_str, std::size_t sz)
    {
        return IP_address { std::string_view { addr_str, sz } };
    }


    inline IP_address operator""_ipv4(unsigned long long raw_addr)
    {
        return IP_address { static_cast<std::uint32_t>(raw_addr) };
    }

} // namespace Navtech::Networking

#endif // IP_ADDRESS_H