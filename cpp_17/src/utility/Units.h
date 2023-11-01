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

#ifndef UNITS_H
#define UNITS_H

#include <cstdint>

#include "Angle.h"
#include "Percentage.h"
#include "Memory_types.h"

namespace Navtech::Unit {

    using Bin               = std::uint16_t;
    using Azimuth           = std::uint16_t;
    using Azimuth_num       = Azimuth;
    using Encoder_step      = std::uint16_t;

    using Volt              = float;
    using Amp               = float;
    using mHz               = std::uint16_t;

    using Metre             = float;
    using Metres_per_sec    = float;

} // namespace Navtech::Unit


// User-defined literals
//
constexpr inline Navtech::Unit::Bin operator""_bins(unsigned long long val)
{
    return static_cast<Navtech::Unit::Bin>(val);
}


constexpr inline Navtech::Unit::Azimuth_num operator""_azimuths(unsigned long long val)
{
    return static_cast<Navtech::Unit::Azimuth_num>(val);
}


constexpr inline Navtech::Unit::Encoder_step operator""_steps(unsigned long long val)
{
    return static_cast<Navtech::Unit::Encoder_step>(val);
}


constexpr inline Navtech::Unit::Volt operator""_volts(unsigned long long val)
{
    return static_cast<Navtech::Unit::Volt>(val);
}


constexpr inline Navtech::Unit::Volt operator""_volts(long double val)
{
    return static_cast<Navtech::Unit::Volt>(val);
}


constexpr inline Navtech::Unit::Amp operator""_amps(unsigned long long val)
{
    return static_cast<Navtech::Unit::Amp>(val);
}


constexpr inline Navtech::Unit::Amp operator""_amps(long double val)
{
    return static_cast<Navtech::Unit::Amp>(val);
}


constexpr inline Navtech::Unit::mHz operator""_mHz(unsigned long long val)
{
    return static_cast<Navtech::Unit::mHz>(val);
}


constexpr inline Navtech::Unit::Amp operator""_Hz(unsigned long long val)
{
    return static_cast<Navtech::Unit::mHz>(val * 1000);
}


constexpr inline Navtech::Unit::Amp operator""_Hz(long double val)
{
    return static_cast<Navtech::Unit::mHz>(val * 1000.0f);
}


constexpr inline Navtech::Unit::Metre operator""_m(unsigned long long val)
{
    return static_cast<Navtech::Unit::Metre>(val);
}


constexpr inline Navtech::Unit::Metre operator""_m(long double val)
{
    return static_cast<Navtech::Unit::Metre>(val);
}


constexpr inline Navtech::Unit::Metres_per_sec operator""_mps(unsigned long long val)
{
    return static_cast<Navtech::Unit::Metres_per_sec>(val);
}


constexpr inline Navtech::Unit::Metres_per_sec operator""_mps(long double val)
{
    return static_cast<Navtech::Unit::Metres_per_sec>(val);
}

#endif // UNITS_H