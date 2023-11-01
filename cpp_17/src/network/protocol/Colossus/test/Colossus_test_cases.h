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

#ifndef COLOSSUS_TEST_CASES_H
#define COLOSSUS_TEST_CASES_H

#include "Colossus_test.h"
#include "Colossus_protocol.h"

namespace Navtech::Networking::Colossus_protocol {

    // -----------------------------------------------------------------------------------------------------------------
    // keep_alive

    void on_keep_alive(Message&, Test& test);

    // -----------------------------------------------------------------------------------------------------------------                  
    // configuration_request
    // configuration                

    void on_config(Message&, Test& test);


    // -----------------------------------------------------------------------------------------------------------------                                    
    // contour_update

    void do_contour_update(Message&, Test& test);
    void on_contoured_fft(Message&, Test& test);
    
    // -----------------------------------------------------------------------------------------------------------------
    // start_fft_data               
    // stop_fft_data                
    // fft_data 
    // high_precision_fft_data
    // start_non_contour_fft_data

    void on_fft(Message&, Test& test);
    void on_non_contoured_fft(Message& msg, Test& test);

    void on_high_precision_fft(Message&, Test& test);

    // -----------------------------------------------------------------------------------------------------------------
    // start_health_msgs            
    // stop_health_msgs
    // health 
    //
    void on_health(Message& msg, Test& test);

    // -----------------------------------------------------------------------------------------------------------------
    // recalibrate_rf_health        
    
    // -----------------------------------------------------------------------------------------------------------------
    // start_tracks                 
    // stop_tracks                  
    
    // -----------------------------------------------------------------------------------------------------------------
    // transmit_on                  
    // transmit_off                 

    // -----------------------------------------------------------------------------------------------------------------
    // sector_blanking_update

    void do_sector_blanking_update(Message&, Test& test);
    void on_blanked_fft(Message& msg, Test& test);

    // -----------------------------------------------------------------------------------------------------------------
    // system_restart               
    
    // -----------------------------------------------------------------------------------------------------------------
    // logging_levels_request              
    // logging_levels

    void on_logging_levels(Message&, Test& test);

    // -----------------------------------------------------------------------------------------------------------------
    // set_auto_tune                
    

    // -----------------------------------------------------------------------------------------------------------------
    // set_navigation_configuration
    // navigation_config_request
    // navigation_configuration

    void do_set_nav_config(Message& msg, Test& test);
    void on_navigation_config(Message& msg, Test& test);

    // -----------------------------------------------------------------------------------------------------------------
    // start_nav_data               
    // stop_nav_data                
    // set_nav_threshold            
    // navigation_data              
    // set_nav_range_offset_and_gain

    void do_set_nav_threshold(Message&, Test& test);

    void do_set_nav_offset_and_gain(Message&, Test& test);
    void on_config_gain_and_offset(Message&, Test& test);

    void on_navigation_data(Message&, Test& test);

    // -----------------------------------------------------------------------------------------------------------------
    // calibrate_accelerometer      
    // start_accelerometer          
    // stop_accelerometer           
    // accelerometer_data

    void on_accelerometer_data(Message&, Test& test);

    // -----------------------------------------------------------------------------------------------------------------
    // set_nav_buffer_config        
    // set_nav_bin_operation        
    // navigation_alarm_data

    void on_nav_alarm_data(Message&, Test& test);

    // -----------------------------------------------------------------------------------------------------------------
    // nav_area_rules_request
    // nav_area_rules
    void do_nav_area_rules(Message& msg, Test&);
    void on_nav_area_rules(Message&, Test& test);

    // -----------------------------------------------------------------------------------------------------------------
    // time_server_status_request
    // time_server_status
    void on_time_server_status(Message& msg, Test& test);

} // namespace Navtech::Networking::Colossus_protocol

#endif // COLOSSUS_TEST_CASES_H