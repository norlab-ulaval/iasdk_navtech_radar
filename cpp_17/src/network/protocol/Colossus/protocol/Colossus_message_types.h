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

#ifndef COLOSSUS_MESSAGE_TYPES_H
#define COLOSSUS_MESSAGE_TYPES_H

namespace Navtech::Networking::Colossus_protocol {

    enum class Type : std::uint8_t {
        invalid                        = 0,
        keep_alive                     = 1,
        configuration                  = 10,
        configuration_request          = 20,
        start_fft_data                 = 21,
        stop_fft_data                  = 22,
        start_health_msgs              = 23,
        stop_health_msgs               = 24,
        recalibrate_rf_health          = 25,
        start_tracks                   = 26,
        stop_tracks                    = 27,
        transmit_on                    = 28,
        transmit_off                   = 29,
        fft_data                       = 30,
        high_precision_fft_data        = 31,
        health                         = 40,
        contour_update                 = 50,
        sector_blanking_update         = 51,
        system_restart                 = 76,
        logging_levels                 = 90,
        logging_levels_request         = 100,
        set_auto_tune                  = 110,
        start_nav_data                 = 120,
        stop_nav_data                  = 121,
        set_nav_threshold              = 122,
        navigation_data                = 123,
        set_nav_range_offset_and_gain  = 124,
        calibrate_accelerometer        = 125,
        start_accelerometer            = 126,
        stop_accelerometer             = 127,
        accelerometer_data             = 128,
        start_non_contour_fft_data     = 140,
        set_nav_buffer_config          = 141,
        set_nav_bin_operation          = 142,
        navigation_alarm_data          = 143,
        nav_area_rules                 = 144,
        nav_radar_reset                = 145,
        nav_radar_halt                 = 146,
        navigation_config_request      = 203,
        navigation_configuration       = 204,
        set_navigation_configuration   = 205,
        nav_area_rules_request         = 206,
        time_server_status_request     = 207,
        time_server_status             = 208
    };
}

#endif // COLOSSUS_MESSAGE_TYPES_H