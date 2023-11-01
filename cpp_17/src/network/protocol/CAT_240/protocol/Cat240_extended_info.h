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

#ifndef CAT240_CONFIGURATION_H
#define CAT240_CONFIGURATION_H


#include "net_conversion.h"
#include "Units.h"

using namespace Navtech::Utility;
using namespace Navtech::Unit;

namespace Navtech::Networking::Cat240_protocol {

// DO NOT REMOVE - 
// This ensures correct alignment for all messages
//
#pragma pack(1)

    namespace Health {

        enum Status { unhealthy, warning, healthy, invalid };

        struct Flags { 
            std::uint16_t die_temperature       : 2;
            std::uint16_t soc_temperature       : 2;
            std::uint16_t vco_temperature       : 2;
            std::uint16_t ambient_temperature   : 2;
            std::uint16_t rotation_rate         : 2;
            std::uint16_t packet_rate           : 2;
            std::uint16_t motor_current         : 2;
            std::uint16_t rf_health             : 2;
        };
    } // namespace Health

    // -----------------------------------------------------------------------------------------------------------------
    // Extended_info forms the Special Purpose field in the CAT-240 message
    //
    class Extended_info {
    public:
        inline Time::Real_time::Observation time() const;
        inline void time(const Time::Real_time::Observation& t);

        std::uint16_t azimuths_per_rotation() const     { return to_uint16_host(azi_samples); }
        void azimuths_per_rotation(std::uint16_t val)   { azi_samples = to_uint16_network(val); }

        std::uint16_t range_in_bins() const             { return to_uint16_host(range_bins); }
        void range_in_bins(std::uint16_t val)           { range_bins = to_uint16_network(val); }

        Unit::mHz rotation_speed() const                { return to_uint16_host(rotation_spd); }
        void rotation_speed(Unit::mHz val)              { rotation_spd = to_uint16_network(val); }

        float range_gain() const                        { return to_float_host(gain); }
        void range_gain(float val)                      { gain = to_uint32_network(val); }

        float range_offset() const                      { return to_float_host(offset); }
        void range_offset(float val)                    { offset = to_uint32_network(val); }

        static std::size_t size()                       { return sizeof(Extended_info); }

        inline Health::Flags health() const;
        inline void health(Health::Flags health_status);

        

        inline std::vector<std::uint8_t> to_vector() const;

        // Derived measures
        //
        inline Unit::Degrees degrees_per_azimuth() const;
        inline Unit::Degrees to_angle(Unit::Azimuth az) const;
        inline Unit::Azimuth to_azimuth(Unit::Degrees angle) const;
        inline std::pair<Unit::Degrees, Unit::Degrees> swept_angle(Unit::Azimuth start);

        // Iterators
        //
        std::uint8_t*       begin()   		{ return reinterpret_cast<std::uint8_t*>(this); }
        const std::uint8_t* begin() const	{ return reinterpret_cast<const std::uint8_t*>(this); }
        std::uint8_t*       end()		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(Extended_info)); }
        const std::uint8_t* end() const     { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(Extended_info)); }

        static Extended_info*       overlay_at(std::uint8_t* addr)       { return reinterpret_cast<Extended_info*>(addr); }
        static const Extended_info* overlay_at(const std::uint8_t* addr) { return reinterpret_cast<const Extended_info*>(addr); }

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.
        //
        std::uint8_t  num_bytes     {  static_cast<std::uint8_t>(Extended_info::size()) };
        std::uint32_t seconds       { };
        std::uint32_t subseconds    { };
        std::uint16_t azi_samples   { };
        std::uint16_t range_bins    { };
        std::uint16_t rotation_spd  { };
        std::uint32_t gain          { };
        std::uint32_t offset        { };

        union Health_word {
            Health::Flags health;
            std::uint16_t word;
        };

        std::uint16_t health_status { 0b1010'1010'1010'1010 }; // <= all healthy
    };

    static_assert(sizeof(Extended_info) == 25);

    // -----------------------------------------------------------------------------------------------------------------
    //
    Time::Real_time::Observation Extended_info::time() const
    {
        std::timespec t { };
        t.tv_sec  = to_uint32_host(seconds);
        t.tv_nsec = to_uint32_host(subseconds);

        return Time::Real_time::Observation { t };
    }


    void Extended_info::time(const Time::Real_time::Observation& t)
    {
        auto ntp_time = t.to_ntp();
        seconds       = to_uint32_network(static_cast<std::uint32_t>(ntp_time.tv_sec));
        subseconds    = to_uint32_network(static_cast<std::uint32_t>(ntp_time.tv_nsec));
    }


    Health::Flags Extended_info::health() const
    {
        Health_word status { };
        
        status.word = to_uint16_host(health_status);
        return status.health;
    }
    
    
    void Extended_info::health(Health::Flags health_settings)
    {
        Health_word status { };

        status.health = health_settings;
        health_status = to_uint16_network(status.word);
    }


    inline std::vector<std::uint8_t>& operator<<(std::vector<std::uint8_t>& v, const Extended_info& cfg)
    {
        v.insert(v.end(), cfg.begin(), cfg.end());
        return v;
    }


    std::vector<std::uint8_t> Extended_info::to_vector() const
    {
        return { begin(), end() }; 
    }


    Unit::Degrees Extended_info::degrees_per_azimuth() const   
    { 
        return Unit::Degrees { azimuths_per_rotation() / 360.0f }; 
    }
    
    
    Unit::Degrees Extended_info::to_angle(Unit::Azimuth az) const 
    { 
        return Unit::Degrees { az / (azimuths_per_rotation() / 360.0f) };
    }


    Unit::Azimuth Extended_info::to_azimuth(Unit::Degrees angle) const
    {
        return static_cast<Unit::Azimuth_num>(std::round(angle.to_float() * (azimuths_per_rotation() / 360.0f))); 
    }


    std::pair<Unit::Degrees, Unit::Degrees> Extended_info::swept_angle(Unit::Azimuth start)
    {
        auto start_angle = to_angle(start);
        
        auto end = start + 1;
        if (end == azimuths_per_rotation()) end = 0;
        auto end_angle = to_angle(end);

        return { start_angle, end_angle };
    }
    

// DO NOT REMOVE - 
// This ensures correct alignment for all messages
//
#pragma pack()

} // namespace Navtech::Networking::Cat240_protocol

#endif // CAT240_CONFIGURATION_H