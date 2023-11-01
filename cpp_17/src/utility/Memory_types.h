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

#ifndef MEMORY_TYPES_H
#define MEMORY_TYPES_H

#include <cstdint>

namespace Navtech {

    namespace Unit {

        // ----------------------------------------------------------------------------
        // Memory literals provide a convenient way to specify memory allocations.
        //
        // For example:
        // std::size_t num_bytes = 1_kB; // num_bytes => 1024;
        //
        class Kilobyte {
        public:
            constexpr Kilobyte() = default;
            constexpr Kilobyte(std::uint32_t sz) : value { sz * 1024 }
            {
            }

            constexpr operator std::size_t() const
            {
                return static_cast<std::size_t>(value);
            }

        private:
            std::uint32_t value { };
        };


        class Megabyte {
        public:
            constexpr Megabyte() = default;
            constexpr Megabyte(std::uint32_t sz) : value { sz * 1024 * 1024 }
            {
            }

            constexpr operator std::size_t() const
            {
                return static_cast<std::size_t>(value);
            }

            constexpr operator Kilobyte() const
            {
                return Kilobyte { value / 1024 };
            }

        private:
            std::uint32_t value { };
        };

    } // namespace Unit


    constexpr Unit::Kilobyte operator""_kB(unsigned long long val)
    {
        return Unit::Kilobyte { static_cast<std::uint32_t>(val) };
    }


    constexpr Unit::Megabyte operator""_MB(unsigned long long val)
    {
        return Unit::Megabyte { static_cast<std::uint32_t>(val) };
    }

}  // namespace Navtech


#endif // MEMORY_TYPES_H