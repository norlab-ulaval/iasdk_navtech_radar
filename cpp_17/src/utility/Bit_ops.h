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

#ifndef BIT_OPS_H
#define BIT_OPS_H

#include <cstdint>
#include <utility>

namespace Navtech::Utility {
    // ----------------------------------------------------------------------------
    // Bit_ops are a collection of common bit manipulation operations
    //

    // Creates a std::uint32_t where bit n is set
    //
    inline constexpr
    std::uint32_t bit(std::uint32_t n)
    {
        if (n > ((sizeof(uint32_t) * 8) - 1)) return 0;  // Overflow
        return (1 << n);
    }


    // Creates a std::uint32_t where bits start -> end
    // (inclusive) are set
    //
    inline constexpr
    std::uint32_t bit_range(std::uint32_t start, std::uint32_t end)
    {
        std::uint32_t value { };

        if (start > end) std::swap(start, end);
        if (end > ((sizeof(uint32_t) * 8) - 1)) end = ((sizeof(uint32_t) * 8) - 1);

        for (auto i = start; i <= end; ++i) {
            value |= bit(i);
        }
        return value;
    }


    // Returns true if bit b is set in word
    //
    inline constexpr
    bool is_set(std::uint32_t word, std::uint32_t b)
    {
        return ((word & bit(b)) != 0);
    }

} // namespace Navtech::Utility


#endif // BIT_OPS_H