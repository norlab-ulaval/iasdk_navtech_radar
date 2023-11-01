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

#ifndef NET_FLOAT_H
#define NET_FLOAT_H

#include <cstdint>
#include <array>
#include <vector>
#include <cstring>
#include <optional>

namespace Navtech::Networking {

    using Byte_array_2 = std::array<std::uint8_t, 2>;
    using Byte_array_4 = std::array<std::uint8_t, 4>;

    std::uint16_t to_uint16_network(std::uint16_t host_value);
    std::uint16_t to_uint16_host(std::uint16_t network_value);

    std::uint32_t to_uint32_host(float host_value);
    std::uint32_t to_uint32_network(float host_value);
    float         from_uint32_host(std::uint32_t host_value);
    float         from_uint32_network(std::uint32_t network_value);
    float         to_float_host(std::uint32_t network_value);
    
    std::uint32_t to_uint32_host(std::uint32_t network_value);
    std::uint32_t to_uint32_network(std::uint32_t host_value);

    std::uint64_t to_uint64_host(double host_value);
    std::uint64_t to_uint64_network(double host_value);
    double        from_uint64_host(std::uint64_t host_value);
    double        from_uint64_network(std::uint64_t network_value);

    Byte_array_4 to_byte_array(std::uint32_t value);
    Byte_array_2 to_byte_array(std::uint16_t value);
    std::uint32_t from_byte_array(const Byte_array_4& value);
    std::uint16_t from_byte_array(const Byte_array_2& value);

    template <typename T>
    std::vector<std::uint8_t> to_vector(const T& value)
    {
        std::vector<std::uint8_t> result(sizeof(T));
        std::memcpy(result.data(), &value, sizeof(T));
        return result;
    }


    template <typename T>
    std::optional<T> from_vector_to(const std::vector<std::uint8_t>& vec)
    {
        if (vec.size() < sizeof(T)) return std::nullopt;

        T result { };
        std::memcpy(&result, vec.data(), sizeof(T));
        return result;
    }

} // namespace Navtech::Networking

#endif // NET_FLOAT_H