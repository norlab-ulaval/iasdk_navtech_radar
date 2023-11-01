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

#ifndef PEAK_FINDER_H
#define PEAK_FINDER_H

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <vector>

#include "navigation_defs.h"

namespace Navtech::Navigation {

    // Buffer modes are really only useful with a staring radar
    //
    enum class Buffer_mode { off, average, max };

    struct Target {
        Target(double rng, double pow) : range { rng }, power { pow } { }

        double range;
        double power;
    };


    struct Azimuth_target {
        Azimuth_target(std::uint16_t azi, double ang, std::uint32_t seconds, std::uint32_t split_seconds) :
            azimuth             { azi }, 
            angle               { ang }, 
            ntp_seconds         { seconds }, 
            ntp_split_seconds   { split_seconds }
        {
        }

        std::uint16_t       azimuth { };
        double              angle { };
        std::vector<Target> targets { };
        std::uint32_t       ntp_seconds { };
        std::uint32_t       ntp_split_seconds { };
    };


    class Peak_finder {
    public:
        void set_target_callback(std::function<void(const Azimuth_target&)> fn = nullptr);
        void fft_data_handler(const FFT_data& fft_data);
        void configure(
            const Configuration_data& data,
            const Colossus::Protobuf::ConfigurationData& protobuf_configuration,
            double        threshold,
            std::uint8_t  bins_to_operate_upon,
            std::uint16_t min_bin_to_operate_upon,
            Buffer_mode   mode,
            std::size_t   buf_length,
            std::uint32_t max_peaks_per_azi
        );
        void set_threshold(double thresh);

    private:
        double        threshold             { 0 };
        std::uint8_t  bins_to_operate_on    { 4 };
        std::uint16_t min_bin_to_operate_on { 50 };
        bool          awaiting_rise         { false };

        std::deque<std::vector<double>> data_buffer { };

        Buffer_mode   buffer_mode           { Buffer_mode::off };
        std::size_t   buffer_length         { 10 };
        std::uint32_t max_peaks_per_azimuth { 10 };

        Configuration_data                    configuration          { };
        Colossus::Protobuf::ConfigurationData protobuf_configuration { };

        std::function<void(const Azimuth_target&)> target_callback { nullptr };

        double peak_resolve(
            const std::vector<double>& data,
            std::uint16_t peak_bin,
            std::uint8_t  bins_to_operate_upon
        );
        
        uint16_t find_peak_bin(
            const std::vector<double>& data,
            std::uint16_t start_bin,
            std::uint16_t end_bin,
            std::uint8_t  bins_to_operate_upon
        );

        void find_peaks(
            std::uint16_t azimuth,
            double angle,
            std::uint32_t ntp_seconds,
            std::uint32_t ntp_split_seconds,
            const std::vector<double>& data);
    };

} // namespace Navtech::Navigation

#endif // PEAK_FINDER_H
