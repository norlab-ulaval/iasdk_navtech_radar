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

#ifndef SECTOR_BLANKING_H
#define SECTOR_BLANKING_H

#include <utility>
#include <vector>
#include <initializer_list>
#include <cstdint>
#include <iostream>

#include "Angle.h"

namespace Navtech::Navigation {

    class Sector : private std::pair<Unit::Degrees, Unit::Degrees> {
    public:
        using Pair = std::pair<Unit::Degrees, Unit::Degrees>;

        using Pair::pair;

        void start(const Unit::Degrees& value)
        {
            Pair::first = value;
        }

        Unit::Degrees start() const
        {
            return Pair::first;
        }

        void finish(const Unit::Degrees& value)
        {
            Pair::second = value;
        }

        Unit::Degrees finish() const
        {
            return Pair::second;
        }

        friend std::ostream& operator<<(std::ostream& os, const Sector& s)
        {
            os << "[" << s.start() << ", " << s.finish() << "]";
            return os;
        }
    };


    class Blanking_sector_list {
    public:
        Blanking_sector_list() = default;
        Blanking_sector_list(std::initializer_list<Sector> init);

        bool add(const Sector& sector);
        bool add(Sector&& sector);

        friend std::ostream& operator<<(std::ostream& os, const Blanking_sector_list& bl);

        std::vector<std::uint8_t> to_vector() const;
        void from_vector(const std::vector<std::uint8_t>& buffer);

    private:
        static constexpr std::size_t max_sectors { 8 };

        std::vector<Sector> sectors { };
    };

} // namespace Navtech::Navigation

#endif // SECTOR_BLANKING_H