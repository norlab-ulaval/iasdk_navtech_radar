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

#include <cmath>
#include <cstdint>

#include "Peak_finder.h"
#include "Vector_maths.h"

namespace Navtech::Navigation {

    constexpr std::uint8_t min_bins_to_operate_on = 5;
    constexpr std::uint8_t max_bins_to_operate_on = 15;

    void Peak_finder::set_target_callback(std::function<void(const Azimuth_target&)> fn)
    {
        target_callback = std::move(fn);
    }


    void Peak_finder::fft_data_handler(const FFT_data& fft_data)
    {
        if (fft_data.data.size() != configuration.range_in_bins) return; // We cannot operate on contoured data

        std::vector<double> buffered_data { };

        if (buffer_mode != Buffer_mode::off) {
            data_buffer.emplace_back(fft_data.data.begin(), fft_data.data.end());
            
            if (data_buffer.size() < buffer_length) return;

            buffered_data.resize(configuration.range_in_bins);
            
            if (buffer_mode == Buffer_mode::average) {
                for (auto bin = 0; bin < configuration.range_in_bins; bin++) {
                    auto total = 0.0f;
                    for (std::uint32_t counter = 0; counter < data_buffer.size(); counter++) {
                        auto value = data_buffer[counter][bin] / 2.0f;
                        total += std::pow(10.0f, (value / 10.0f));
                    }
                    buffered_data[bin] = std::log10(total / data_buffer.size()) * 10.0f;
                }
            }
            else {
                double max = 0.0;
                for (std::int32_t bin = 0; bin < configuration.range_in_bins; bin++) {
                    for (std::uint32_t counter = 0; counter < data_buffer.size(); counter++) {
                        auto value = data_buffer[counter][bin] / 2.0f;
                        if (value > max) { max = value; }
                    }
                    buffered_data[bin] = max;
                }
            }
            data_buffer.clear();
        }
        else {
            buffered_data = std::vector<double>(fft_data.data.begin(), fft_data.data.end());
        }

        find_peaks(
            fft_data.azimuth, 
            (fft_data.azimuth * 360.0f) / configuration.encoder_size, 
            fft_data.ntp_seconds, 
            fft_data.ntp_split_seconds, 
            buffered_data
        );
    }


    void Peak_finder::configure(
        const Configuration_data& config,
        const Colossus::Protobuf::ConfigurationData& protobuf,
        double thresh,
        std::uint8_t bins_to_operate_upon,
        std::uint16_t min_bin_to_operate_upon,
        Buffer_mode mode,
        std::size_t buf_length,
        std::uint32_t max_peaks_per_azi
    )
    {
        configuration          = config;
        protobuf_configuration = protobuf;
        threshold              = thresh;
        bins_to_operate_on     = std::max(std::min(bins_to_operate_upon, max_bins_to_operate_on), min_bins_to_operate_on);
        min_bin_to_operate_on  = min_bin_to_operate_upon;
        buffer_mode            = mode;
        buffer_length          = buf_length;
        max_peaks_per_azimuth  = max_peaks_per_azi;
    }


    void Peak_finder::set_threshold(double thresh)
    {
        threshold = thresh;
    }


    void Peak_finder::find_peaks(
        std::uint16_t azimuth,
        double angle,
        std::uint32_t ntp_seconds,
        std::uint32_t ntp_split_seconds,
        const std::vector<double>& data
    )
    {
        awaiting_rise  = false;

        std::uint16_t peak_bin       { 0 };
        std::uint32_t peaks_found    { 0 };
        auto min_bin_to_operate_upon { min_bin_to_operate_on };
        auto minimum_range           { bins_to_operate_on * protobuf_configuration.rangeresolutionmetres() };
        auto maximum_range           { configuration.range_in_bins * protobuf_configuration.rangeresolutionmetres() };
        Azimuth_target target        { azimuth, angle, ntp_seconds, ntp_split_seconds };

        while (peak_bin != configuration.range_in_bins) {
            peak_bin = find_peak_bin(
                data, 
                min_bin_to_operate_upon, 
                configuration.range_in_bins, 
                bins_to_operate_on
            );
            
            min_bin_to_operate_upon = peak_bin + bins_to_operate_on;

            if (peak_bin < configuration.range_in_bins) {
                auto resolvedBin = peak_resolve(data, peak_bin, bins_to_operate_on);
                auto range {
                    (resolvedBin * configuration.range_gain * protobuf_configuration.rangeresolutionmetres()) +
                    configuration.range_offset
                };

                if (std::isinf(range) || range < minimum_range || range > maximum_range) continue;
                
                target.targets.emplace_back(range, data[peak_bin]);
                peaks_found++;
            }
            if (peaks_found >= max_peaks_per_azimuth) break;
        }

        if (peaks_found > 0 && target_callback != nullptr) target_callback(target);
    }


    std::uint16_t Peak_finder::find_peak_bin(
        const std::vector<double>& data,
        std::uint16_t start_bin,
        std::uint16_t end_bin,
        std::uint8_t  bins_to_operate_upon
    )
    {
        if (start_bin > (end_bin - bins_to_operate_upon)) return end_bin;

        for (auto peakBin = start_bin; peakBin <= end_bin - bins_to_operate_upon; peakBin++) {
            if (awaiting_rise && data[peakBin] > data[peakBin + 1]) {
                continue;
            }
            else if (awaiting_rise) {
                awaiting_rise = false;
            }
            if (data[peakBin] > threshold && data[peakBin + 1] < data[peakBin]) {
                awaiting_rise = true;
                return peakBin;
            }
        }

        return end_bin;
    }


    double Peak_finder::peak_resolve(
        const std::vector<double>& data,
        std::uint16_t peak_bin,
        std::uint8_t  bins_to_operate_upon
    )
    {
        const std::uint8_t bins_to_offset { static_cast<uint8_t>((bins_to_operate_upon - 1) / 2) };
        double x[max_bins_to_operate_on]  { 0.0 };
        double y[max_bins_to_operate_on]  { 0.0 };
        auto index                        { 0 };
        auto startValue                   { 0 - bins_to_offset };

        for (index = 0; index < bins_to_operate_upon; index++) {
            x[index] = (float)startValue++;
        }

        auto startBin { peak_bin - bins_to_offset };

        for (index = 0; index < bins_to_operate_upon; index++) {
            y[index] = data[startBin + index];
        }

        double Sx  { };
        double Sx2 { };
        double Sx3 { }; 
        double Sx4 { };
        double x2[max_bins_to_operate_on] { };
        double x3[max_bins_to_operate_on] { };
        double x4[max_bins_to_operate_on] { };

        Vector_maths::scalar_sum(x, bins_to_operate_upon, Sx);
        Vector_maths::scalar_square(x, bins_to_operate_upon, Sx2);
        Vector_maths::vector_cube(x, bins_to_operate_upon, x3);
        Vector_maths::scalar_sum(x3, bins_to_operate_upon, Sx3);
        Vector_maths::vector_square(x, bins_to_operate_upon, x2);
        Vector_maths::vector_multiply(x2, x2, bins_to_operate_upon, x4);
        Vector_maths::scalar_sum(x4, bins_to_operate_upon, Sx4);

        double Sy   { };
        double Sxy  { };
        double Sx2y { };
        double xy[max_bins_to_operate_on]  { };
        double x2y[max_bins_to_operate_on] { };

        Vector_maths::scalar_sum(y, bins_to_operate_upon, Sy);
        Vector_maths::vector_multiply(x, y, bins_to_operate_upon, xy);
        Vector_maths::scalar_sum(xy, bins_to_operate_upon, Sxy);
        Vector_maths::vector_multiply(x2, y, bins_to_operate_upon, x2y);
        Vector_maths::scalar_sum(x2y, bins_to_operate_upon, Sx2y);

        double A[4] { Sx2, Sx3, Sx4, Sx2y };
        double B[4] { Sx, Sx2, Sx3, Sxy };
        double C[4] { (double)bins_to_operate_upon, Sx, Sx2, Sy };

        double F = C[0] / A[0];

        for (index = 0; index <= 3; index++) {
            C[index] = C[index] - (F * A[index]);
        }

        F = B[0] / A[0];

        for (index = 0; index <= 3; index++) {
            B[index] = B[index] - (F * A[index]);
        }

        F = C[1] / B[1];

        for (index = 1; index <= 3; index++) {
            C[index] -= F * B[index];
        }

        double b2  { C[3] / C[2] };
        double b1  {  (B[3] - B[2] * b2) / B[1] };

        return -b1 / (2
         * b2) + startBin - (double)(0 - bins_to_offset);
    }

} // namespace Navtech::Navigation
