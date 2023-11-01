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

#ifdef __linux
#include <arpa/inet.h>
#else
#include "winsock.h"
#endif

#include <cstring>

#include "net_conversion.h"


namespace Navtech::Networking {
    
    union float_uint32_map {
        float         as_float;
        std::uint32_t as_uint;
        std::uint8_t  as_array[4];
    };


    union double_uint64_map {
        double        as_double;
        std::uint64_t as_uint;
        std::uint32_t as_array[2];
    };

    std::uint16_t to_uint16_network(std::uint16_t host_value)
    {
        return htons(host_value);
    }


    std::uint16_t to_uint16_host(std::uint16_t network_value)
    {
        return ntohs(network_value);
    }


    std::uint32_t to_uint32_host(float host_value)
    {
        float_uint32_map u { };
        u.as_float = host_value;
        return u.as_uint;
    }


    std::uint32_t to_uint32_network(float host_value)
    {
        float_uint32_map u { };
        u.as_float = host_value;
        return htonl(u.as_uint);
    }


    float from_uint32_host(std::uint32_t host_value)
    {
        float_uint32_map u { };
        u.as_uint = host_value;
        return u.as_float;
    }


    float from_uint32_network(std::uint32_t network_value)
    {
        float_uint32_map u { };
        u.as_uint = ntohl(network_value);
        return u.as_float;
    }


    float to_float_host(std::uint32_t network_value)
    {
        return from_uint32_network(network_value);
    }
    

    std::uint32_t to_uint32_host(std::uint32_t network_value)
    {
        return ntohl(network_value);
    }


    std::uint32_t to_uint32_network(std::uint32_t host_value)
    {
        return htonl(host_value);
    }


    Byte_array_4 to_byte_array(std::uint32_t value)
    {
        Byte_array_4 buffer { };
        std::memcpy(&buffer[0], &value, sizeof(std::uint32_t));
        
        return buffer;
    }


    std::uint32_t from_byte_array(const Byte_array_4& value)
    {
        std::uint32_t output { };
        std::memcpy(&output, &value[0], sizeof(output));
        
        return output;
    }


    Byte_array_2 to_byte_array(std::uint16_t value)
    {
        Byte_array_2 buffer { };
        std::memcpy(&buffer[0], &value, sizeof(std::uint16_t));
        
        return buffer;
    }


    std::uint16_t from_byte_array(const Byte_array_2& value)
    {
        std::uint16_t output { };
        std::memcpy(&output, &value[0], sizeof(output));
        
        return output;
    }


    std::uint64_t to_uint64_host(double host_value)
    {
        double_uint64_map u { };
        u.as_double = host_value;
        return u.as_uint;
    }


    std::uint64_t to_uint64_network(double host_value)
    {
        double_uint64_map u { };
        u.as_double = host_value;
        u.as_array[0] = to_uint32_network(u.as_array[0]);
        u.as_array[1] = to_uint32_network(u.as_array[1]);
        return u.as_uint;
    }


    double from_uint64_host(std::uint64_t host_value)
    {
        double_uint64_map u { };
        u.as_uint = host_value;
        return u.as_double;
    }


    double from_uint64_network(std::uint64_t network_value)
    {
        double_uint64_map u { };
        u.as_uint = network_value;
        u.as_array[0] = to_uint32_host(u.as_array[0]);
        u.as_array[1] = to_uint32_host(u.as_array[1]);
        return u.as_double;
    }
    
} // namespace Navtech::Networking