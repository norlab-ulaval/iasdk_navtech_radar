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

#ifndef NAVIGATION_DEFS_H
#define NAVIGATION_DEFS_H

#include "pointer_types.h"
#include "configurationdata.pb.h"
#include "health.pb.h"


namespace Navtech::Navigation {

    constexpr std::uint32_t range_multiplier       { 1'000'000 };
    constexpr float range_multiplier_float         { 1'000'000.0f };
    constexpr std::uint32_t nav_data_record_length { sizeof(std::uint32_t) + sizeof(std::uint16_t) };

    class Blanking_sector_list;

    struct FFT_data {
        using Pointer = shared_owner<FFT_data>;
    
        std::uint16_t azimuth { 0 };
        std::uint16_t sweep_counter { 0 };
        std::uint32_t ntp_seconds { 0 };
        std::uint32_t ntp_split_seconds { 0 };
        std::vector<std::uint8_t> data;
    };


    struct Navigation_data {
        using Pointer = shared_owner<Navigation_data>;
        
        double angle { 0.0 };
        std::uint16_t azimuth { 0 };
        std::uint32_t ntp_seconds { 0 };
        std::uint32_t ntp_split_seconds { 0 };
        std::vector<std::tuple<float, std::uint16_t>> peaks;
    };


    struct Configuration_data {
        using Pointer         = shared_owner<Configuration_data>;
        using ProtobufPointer = shared_owner<Colossus::Protobuf::ConfigurationData>;

        std::uint16_t   azimuth_samples         {  };
        std::uint16_t   encoder_size            {  };
        double          bin_size                {  };
        std::uint16_t   range_in_bins           {  };
        std::uint16_t   expected_rotation_rate  {  };
        float           range_gain              {  };
        float           range_offset            {  };
    };

   
    struct Navigation_config {
        using Pointer = shared_owner<Navigation_config>;

        std::uint16_t bins_to_operate_on;
        std::uint16_t min_bin;
        float navigation_threshold;
        std::uint32_t max_peaks_per_azimuth;
    };

} // namespace Navtech::Navigation

#endif // NAVIGATION_DEFS_H