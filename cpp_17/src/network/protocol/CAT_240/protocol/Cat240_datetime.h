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

#ifndef CAT240_DATETIME_H
#define CAT240_DATETIME_H

#include "uint24_t.h"
#include "Time_utils.h"

namespace Navtech::Networking::Cat240_protocol {

    class Time_of_day : public uint24_t {
    public:
        Time_of_day(const Time::Real_time::Observation& time)
        {
            float since_midnight { };
            since_midnight += time.hour() * 3600.0f;
            since_midnight += time.minute() * 60.0f;
            since_midnight += time.second();
            since_midnight += time.milliseconds() / 1000.0f;
            since_midnight += time.microseconds() / 1000'000.0f;
            
            store(static_cast<std::uint32_t>(since_midnight * (1 << decimal_places)));
        }

        Time::Real_time::Observation to_observation() const
        {
            auto duration = static_cast<std::uint32_t>(to_float() * 1000);
            return Time::Real_time::Observation { Time::to_msec_duration(duration) };
        }

        static constexpr std::size_t size() { return sizeof(Time_of_day); } 

        // Memory overlays
        //
        static Time_of_day*       overlay_at(std::uint8_t* addr)       { return reinterpret_cast<Time_of_day*>(addr); }
        static const Time_of_day* overlay_at(const std::uint8_t* addr) { return reinterpret_cast<const Time_of_day*>(addr); }

    protected:
        float to_float() const
        {
            return static_cast<float>(load()) / (1 << decimal_places);
        }

    private:
        static constexpr int decimal_places { 7 };
    };

} // namespace Navtech::Networking::Cat240_protocol


#endif // CAT240_DATETIME_H