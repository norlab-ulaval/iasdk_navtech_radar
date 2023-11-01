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

#include <regex>
#include <sstream>
#include <stdexcept>

#ifdef __linux
#include <arpa/inet.h>
#else
#include "winsock.h"
#endif

#include "IP_address.h"
#include "net_conversion.h"

namespace Navtech::Networking {

    IP_address::IP_address(std::string_view ip_addr_str)
    {
        validate(ip_addr_str);
        parse_string(ip_addr_str);
    }


    IP_address::IP_address(std::uint32_t raw_addr) :
        address { raw_addr }
    {
    }


    IP_address::IP_address(std::uint32_t raw_addr, Endian endian) :
        address { (endian == Endian::host) ? raw_addr : ntohl(raw_addr) }
    {
    }


     IP_address::IP_address(const sockaddr& socket_addr)
    {
        union Address{
            struct sockaddr sa;
            struct sockaddr_in sa_ipv4;
        };

        Address addr { socket_addr };
        address.word = to_uint32_host(addr.sa_ipv4.sin_addr.s_addr);
    }


    IP_address::IP_address(const sockaddr_in& socket_addr) :
        address { to_uint32_host(socket_addr.sin_addr.s_addr) }
    {
    }


    IP_address::operator std::uint32_t() const
    {
        return to_host_endian();
    }


    std::uint32_t IP_address::to_host_endian() const
    {
        return address.word;
    }


    std::uint32_t IP_address::to_network_endian() const
    {
        return htonl(address.word);
    }


    std::uint32_t IP_address::value_as(Endian endian) const
    {
        if (endian == Endian::host) return to_host_endian();
        else                        return to_network_endian();
    }


    Byte_array IP_address::to_byte_array() const
    {
        Byte_array bytes { };
        std::memcpy(&bytes, &address, sizeof(address));

        return bytes;
    }
    

    IP_address& IP_address::operator=(std::uint32_t ip_addr)
    {
        address.word = ip_addr;
        return *this;
    }


    IP_address& IP_address::operator=(std::string_view ip_addr_str)
    {
        validate(ip_addr_str);
        parse_string(ip_addr_str);
        return *this;
    }


    IP_address IP_address::operator ~() const
    {
        return IP_address { ~address.word };
    }


    IP_address IP_address::operator&(const IP_address& rhs) const
    {
        return IP_address { (this->address.word & rhs.address.word) };
    }


    IP_address& IP_address::operator&=(const IP_address& rhs)
    {
        this->address.word &= rhs.address.word;
        return *this;
    }


    IP_address IP_address::operator&(std::uint32_t rhs) const
    {
        return IP_address { (this->address.word & rhs) };
    }


    IP_address& IP_address::operator&=(std::uint32_t rhs)
    {
        this->address.word &= rhs;
        return *this;
    }


    IP_address operator&(std::uint32_t lhs, const IP_address& rhs)
    {
        return IP_address { (lhs & rhs.address.word) };
    }
    
    //

    IP_address IP_address::operator|(const IP_address& rhs) const
    {
        return IP_address { (this->address.word | rhs.address.word) };
    }


    IP_address& IP_address::operator|=(const IP_address& rhs)
    {
        this->address.word |= rhs.address.word;
        return *this;
    }


    IP_address IP_address::operator|(std::uint32_t rhs) const
    {
        return IP_address { (this->address.word | rhs) };
    }


    IP_address& IP_address::operator|=(std::uint32_t rhs)
    {
        this->address.word |= rhs;
        return *this;
    }


    IP_address operator|(std::uint32_t lhs, const IP_address& rhs)
    {
        return IP_address { (lhs | rhs.address.word) };
    }


    bool IP_address::operator==(const IP_address& rhs) const
    {
        return (this->address.word == rhs.address.word);
    }


    bool IP_address::operator==(std::uint32_t rhs) const
    {
        return (this->address.word == rhs);
    }


    bool IP_address::operator!=(const IP_address& rhs) const
    {
        return !operator==(rhs);
    }


    bool IP_address::operator!=(std::uint32_t rhs) const
    {
        return !operator==(rhs);
    }


    std::string IP_address::to_string() const
    {
        std::ostringstream os { };

        os << static_cast<int>(address.byte[3]);
        os << ".";
        os << static_cast<int>(address.byte[2]);
        os << ".";
        os << static_cast<int>(address.byte[1]);
        os << ".";
        os << static_cast<int>(address.byte[0]);

        return os.str();
    }


    void IP_address::parse_string(std::string_view ip_addr_str)
    {
        using std::istringstream;
        using std::string;
        using std::stoi;
        using std::getline;

        istringstream stream { string { ip_addr_str } };
        string str { };

        for (int i { 3 }; i >= 0; --i) {
            if (getline(stream, str, '.')) {
                address.byte[i] = stoi(str, nullptr, 0);
            }
        }
    }


    void IP_address::validate(std::string_view ip_addr_str)
    {
        using std::string;
        using std::regex;
        using std::regex_match;

        string check_this { ip_addr_str };

        // For more information on this regex see:
        // https://stackoverflow.com/questions/5284147/validating-ipv4-addresses-with-regexp
        //
        regex  ip_format  { R"(^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$)" };

        if (!regex_match(check_this, ip_format)) throw std::out_of_range { "Invalid IP address" };
    }


    IP_address IP_address::localhost()
    {
        return "127.0.0.1"_ipv4;
    }


    IP_address IP_address::any()
    {
        return "0.0.0.0"_ipv4;
    }

} // namespace Navtech::Networking