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

#ifndef TRIPLE_OCTET_H
#define TRIPLE_OCTET_H

#include <cstdint>
#include <array>
#include <algorithm>

#include "net_conversion.h"

namespace Navtech {

    class uint24_t {
    public:
        uint24_t() = default;

        uint24_t(std::uint32_t init)
        {
            store(init);
        }

        uint24_t(const std::vector<std::uint8_t>& v)
        {
            std::copy_n(std::begin(v), value.size(), std::begin(value));
        }

        std::uint32_t to_uint32() const
        {
            return load();
        }

        std::array<std::uint8_t, 3> host_endian() const
        {
            return { value[2], value[1], value[0] };
        }

        std::array<std::uint8_t, 3> network_endian() const
        {
            return value;
        }

        std::vector<std::uint8_t> to_vector() const
        {
            return { std::begin(value), std::end(value) };
        }

        std::vector<std::uint8_t> relinquish()
        {
            auto result = to_vector();
            std::array<std::uint8_t, 3> { }.swap(value);
            return result;
        }

        // Operator overloads
        //
        uint24_t& operator=(const std::uint32_t& rhs)
        {
            store(rhs);
            return *this;
        }

        // Iterators
        //
        std::uint8_t*       begin()   		{ return reinterpret_cast<std::uint8_t*>(this); }
        const std::uint8_t* begin() const	{ return reinterpret_cast<const std::uint8_t*>(this); }
        std::uint8_t*       end()		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(uint24_t)); }
        const std::uint8_t* end() const     { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(uint24_t)); }

        // Memory overlays
        //
        static uint24_t*       overlay_at(std::uint8_t* addr)       { return reinterpret_cast<uint24_t*>(addr); }
        static const uint24_t* overlay_at(const std::uint8_t* addr) { return reinterpret_cast<const uint24_t*>(addr); }

    protected:
        union uint32_bytes {
            std::uint32_t word;
            std::uint8_t  byte[4];
        };

        void store(std::uint32_t host_value)
        {
            // 32-bit value host-endian to 
            // 24-bit network-endian
            //
            uint32_bytes network_endian { Networking::to_uint32_network(host_value) };
            value[0] = network_endian.byte[1];
            value[1] = network_endian.byte[2];
            value[2] = network_endian.byte[3];
        }

        std::uint32_t load() const
        {
            // 24-bit value network-endian to 
            // 32-bit host-endian
            //
            uint32_bytes network_endian { };
            network_endian.byte[1] = value[0];
            network_endian.byte[2] = value[1];
            network_endian.byte[3] = value[2];

            return Networking::to_uint32_host(network_endian.word);
        }

    private:
        std::array<std::uint8_t, 3> value { };
    };


    static_assert(sizeof(uint24_t) == 3);


    // -----------------------------------------------------------------------------------------------------------------
    //
    inline std::vector<std::uint8_t> operator<<(std::vector<std::uint8_t>& v, const uint24_t& tod)
    {
        auto tod_vector = tod.to_vector();
        v.insert(v.end(), tod_vector.begin(), tod_vector.end());
        return v;
    }


    inline std::vector<std::uint8_t> operator<<(std::vector<std::uint8_t>& v, uint24_t&& tod)
    {
        auto tod_vector = tod.to_vector();
        v.insert(v.end(), tod_vector.begin(), tod_vector.end());
        return v;
    }

    inline void operator>>(const std::vector<std::uint8_t>& v, uint24_t& tod)
    {
        tod = uint24_t { v };
    } 

} // namespace Navtech::Utility

#endif // TRIPLE_OCTET_H