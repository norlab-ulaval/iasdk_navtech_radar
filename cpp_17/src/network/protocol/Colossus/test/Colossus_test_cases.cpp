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

#include "Colossus_test_cases.h"

#include <array>

#include "Log.h"
#include "Time_utils.h"
#include "Statistical_value.h"
#include "Protobuf_helpers.h"
#include "net_conversion.h"
#include "float_equality.h"
#include "Navigation_area_rule.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

using Navtech::Utility::Statistical_value;
using namespace Navtech::Time;


namespace Navtech::Networking::Colossus_protocol {

    // -----------------------------------------------------------------------------------------------------------------
    // keep_alive

    void on_keep_alive(Message&, Test& test)
    {
        static Monotonic::Observation t_prev { Monotonic::now() };
        static Monotonic::Observation t_now  { Monotonic::now() };

        stdout_log << "Received keep-alive message" << endl;

        auto delta = t_now - t_prev;

        if (delta > 5.1_sec) {
            stdout_log << "Keep-alive message arrived too late: ";
            stdout_log << delta.to_string();
            stdout_log << endl;

            test.result(Test::Result::fail);
        }

        t_prev = t_now;
        test.result(Test::pass);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // start_health_msgs            
    // stop_health_msgs
    // health 
    //
    void on_health(Message&, Test& test)
    {
        static Monotonic::Observation t_prev { Monotonic::now() };
        static Monotonic::Observation t_now  { Monotonic::now() };

        stdout_log << "Received Health message" << endl;

        auto delta = t_now - t_prev;

        if (delta > 5.1_sec) {
            stdout_log << "Health message arrived too late: ";
            stdout_log << delta.to_string();
            stdout_log << endl;

            test.result(Test::Result::fail);
        }

        t_prev = t_now;
        test.result(Test::pass);
    }


    // -----------------------------------------------------------------------------------------------------------------                  
    // configuration_request
    // configuration                

    void on_config(Message&, Test& test)
    {
        test.result(Test::pass);
    }        
    
    // -----------------------------------------------------------------------------------------------------------------
    // start_fft_data               
    // stop_fft_data                
    // fft_data 
    // high_precision_fft_data
    // start_non_contour_fft_data

    namespace Contoured_FFT {

        bool rotated_once(Azimuth_num azimuth)
        {
            static bool has_rotated_once { };
            static Azimuth_num prev { 0 };
            
            if (has_rotated_once) return true;
            if (azimuth < prev) has_rotated_once = true;
            prev = azimuth;

            return has_rotated_once;
        }


        bool completed_full_rotation(Azimuth_num azimuth)
        {
            if (!rotated_once(azimuth)) return false;

            bool has_completed_rotation { false };
            static Azimuth_num prev { };

            if (azimuth < prev) has_completed_rotation = true;
            prev = azimuth;

            return has_completed_rotation;
        }


        bool lost_packet(std::uint16_t counter, unsigned long packet_count)
        {
            static bool first_update { true };
            static std::uint16_t prev { };

            if (first_update) {
                prev = counter;
                first_update = false;
                return false;
            }

            if (counter != static_cast<std::uint16_t>(prev + 1)) {
                stdout_log << Logging_level::error << "Packets lost! "
                        << "packet [" << packet_count << "] "
                        << "current sweep counter [" << counter << "] "
                        << "previous [" << prev << "] "
                        << pop_level << endl;
                
                return true;
            }

            prev = counter;
            return false;
        }
    } // namespace Contoured_FFT


    void on_fft(Message& msg, Test& test)
    {
        using namespace Navtech::Networking;
        using namespace Navtech::Time::Monotonic;
        using namespace Contoured_FFT;
        using Navtech::Protobuf::from_vector_into;

        static unsigned long packet_count { };
        static unsigned rotations { };
        static Observation t0 { now() };
        static Statistical_value<double, 10> packet_rate { };

        ++packet_count;

        auto fft  = msg.view_as<Colossus_protocol::FFT_data>();
        auto data = fft->to_vector();

        if (lost_packet(fft->sweep_counter(), packet_count)) {
            test.result(Test::fail);
        };

        if (!completed_full_rotation(fft->azimuth())) return;

        ++rotations;
        auto t1 = now();
        auto rotation_period = t1 - t0;
        
        packet_rate = packet_count / rotation_period.to_sec();

        if (rotations % 10 == 0) {
            stdout_log  << "Rotation [" << rotations << "] "
                        << "FFT size [" << data.size() << "] "
                        << "average packet rate [" << packet_rate.mean() << "] "
                        << endl;           
        }

        packet_count = 0;
        t0 = t1;
    }
    

    namespace Non_contoured_FFT {

        bool rotated_once(Azimuth_num azimuth)
        {
            static bool has_rotated_once { };
            static Azimuth_num prev { 0 };
            
            if (has_rotated_once) return true;
            if (azimuth < prev) has_rotated_once = true;
            prev = azimuth;

            return has_rotated_once;
        }


        bool completed_full_rotation(Azimuth_num azimuth)
        {
            if (!rotated_once(azimuth)) return false;

            bool has_completed_rotation { false };
            static Azimuth_num prev { };

            if (azimuth < prev) has_completed_rotation = true;
            prev = azimuth;

            return has_completed_rotation;
        }


        bool lost_packet(std::uint16_t counter, unsigned long packet_count)
        {
            static bool first_update { true };
            static std::uint16_t prev { };

            if (first_update) {
                prev = counter;
                first_update = false;
                return false;
            }

            if (counter != static_cast<std::uint16_t>(prev + 1)) {
                stdout_log << Logging_level::error << "Packets lost! "
                        << "packet [" << packet_count << "] "
                        << "current sweep counter [" << counter << "] "
                        << "previous [" << prev << "] "
                        << pop_level << endl;
                
                return true;
            }

            prev = counter;
            return false;
        }

    } // namespace Non_contoured_FFT

    
    void on_high_precision_fft(Message&, Test& test)
    {
        test.result(Test::pass);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // recalibrate_rf_health        
    
    // -----------------------------------------------------------------------------------------------------------------
    // start_tracks                 
    // stop_tracks                  
    
    // -----------------------------------------------------------------------------------------------------------------
    // transmit_on                  
    // transmit_off                 

    // -----------------------------------------------------------------------------------------------------------------                                    
    // contour_update

    namespace Contour_map {
        constexpr std::uint16_t    contour_len { 100 };
        std::vector<std::uint16_t> contour_map { };
    }

    void do_contour_update(Message& msg, Test& test)
    {
        using namespace Contour_map;
        using namespace Utility;
        using namespace std;

        contour_map.resize(360, to_uint16_network(contour_len));

        vector<uint8_t> payload { };
        payload.resize(720);

        memcpy(payload.data(), contour_map.data(), 720);

        msg.append(std::move(payload));

        test.result(Test::pass);
    }            
    

    void on_contoured_fft(Message& msg, Test& test)
    {
        using namespace Contour_map;

        auto fft  = msg.view_as<Colossus_protocol::FFT_data>();
        auto data = fft->to_vector();

        if (data.size() != contour_len) {
            test.result(Test::fail);
            return;
        }

        test.result(Test::pass);
    }

         
    void on_non_contoured_fft(Message& msg, Test& test)
    {
        using namespace Contour_map;

        auto fft  = msg.view_as<Colossus_protocol::FFT_data>();
        auto data = fft->to_vector();

        if (data.size() != test.config().range_in_bins()) {
            test.result(Test::fail);
            return;
        }

        test.result(Test::pass);
    }


    // -----------------------------------------------------------------------------------------------------------------
    // sector_blanking_update

    namespace Sector_blanking_test {

        class Blanking_sector {
        public:
            Blanking_sector(float start_angle, float end_angle)
            {
                start(start_angle);
                end(end_angle);
            }

            void  start(float val)  { start_val = to_uint32_network(val * 10.0f); }
            float start() const     { return to_float_host(start_val) / 10.0f; }

            void  end(float val)    { end_val = to_uint32_network(val * 10.0f); }
            float end() const       { return to_float_host(end_val) / 10.0f; }

            std::vector<std::uint8_t> to_vector() const
            {
                std::vector<std::uint8_t> result { };
                result.resize(sizeof(std::uint32_t) * 2);

                std::memcpy(&result[0], &start_val, sizeof(std::uint32_t));
                std::memcpy(&result[4], &end_val,   sizeof(std::uint32_t));

                return result;
            }

        private:
            std::uint32_t start_val;
            std::uint32_t end_val;
        };


        Blanking_sector s1 { 0.0f, 90.0f };
        Blanking_sector s2 { 180.0f, 270.0f };
    }

    void do_sector_blanking_update(Message& msg, Test& test)
    {
        using namespace Sector_blanking_test;
        using namespace std;

        auto s1_vec = s1.to_vector();
        auto s2_vec = s2.to_vector();

        vector<uint8_t> sectors { };
        sectors.push_back(2);
        sectors.insert(sectors.end(), s1_vec.begin(), s1_vec.end());
        sectors.insert(sectors.end(), s2_vec.begin(), s2_vec.end());

        msg.append(std::move(sectors));

        test.result(Test::pass);
    } 
    

    void on_blanked_fft(Message& msg, Test& test)
    {
        auto fft  = msg.view_as<Colossus_protocol::FFT_data>();
        auto data = fft->to_vector();
        
        auto num_az = test.config().azimuth_samples();

        // Check sector1
        //
        if (fft->azimuth() > 0 && fft->azimuth() < (num_az / 4) ) {
            if (data.size() != 0) test.result(Test::fail);
        }

        // Check sector2
        //
        if (fft->azimuth() > (num_az / 2) && fft->azimuth() < ((num_az * 3) / 4)) {
            if (data.size() != 0) test.result(Test::fail);
        }

        test.result(Test::pass);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // system_restart               
    
    // -----------------------------------------------------------------------------------------------------------------
    // logging_levels_request              
    // logging_levels

    void on_logging_levels(Message&, Test& test)
    {
        test.result(Test::pass);
    }     
    
    // -----------------------------------------------------------------------------------------------------------------
    // set_auto_tune                
    
    // -----------------------------------------------------------------------------------------------------------------
    // set_navigation_configuration
    // navigation_config_request
    // navigation_configuration

    namespace Nav_config {

        Navigation_config nav_cfg { };
    }


    void do_set_nav_config(Message& msg, Test& test)
    {
        using namespace Nav_config;

        nav_cfg.min_bin_to_operate_on(100);
        nav_cfg.bins_to_operate_on(50);
        nav_cfg.navigation_threshold(0.5f);
        nav_cfg.max_peaks_per_azimuth(5);

        msg.append(nav_cfg);

        test.result(Test::pass);
    }


    void on_navigation_config(Message& msg, Test& test)
    {
        using namespace Utility;
        using namespace Nav_config;

        auto cfg = msg.view_as<Navigation_config>();
        
        if (cfg->min_bin_to_operate_on() != nav_cfg.bins_to_operate_on())                     test.result(Test::fail);
        if (cfg->bins_to_operate_on() != nav_cfg.bins_to_operate_on())                        test.result(Test::fail);
        if (!essentially_equal(cfg->navigation_threshold(), nav_cfg.navigation_threshold()))  test.result(Test::fail);
        if (cfg->max_peaks_per_azimuth() != nav_cfg.max_peaks_per_azimuth())                  test.result(Test::fail);

        test.result(Test::pass);
    }


    // -----------------------------------------------------------------------------------------------------------------
    // start_nav_data               
    // stop_nav_data                
    // set_nav_threshold            
    // navigation_data              
    // set_nav_range_offset_and_gain

    void do_set_nav_threshold(Message& msg, Test& test)
    {
        Set_navigation_threshold header { };
        header.threshold(0.5f);
        msg.append(header);

        test.result(Test::pass);
    }


    void do_set_nav_offset_and_gain(Message& msg, Test& test)
    {
        Set_navigation_gain_and_offset header { };
        header.gain(1.0f);
        header.offset(0.31);

        msg.append(header);

        test.result(Test::pass);
    }


    void on_config_gain_and_offset(Message& msg, Test& test)
    {
        using namespace Utility;

        auto cfg = msg.view_as<Configuration>();
        if (!essentially_equal(cfg->range_gain(), 1.0f))      test.result(Test::fail);
        if (!essentially_equal(cfg->range_offset(), 0.31f))   test.result(Test::fail);

        test.result(Test::pass);
    }


    void on_navigation_data(Message& msg, Test& test)
    {
        using namespace Nav_config;

        auto nav        = msg.view_as<Navigation_data>();
        auto nav_pairs  = nav->to_vector();

        if (nav_pairs.size() != 0) {
            if (nav_pairs.size() % sizeof(Navigation_pair) != 0) {
                stdout_log << "Invalid navigation data payload size. " << endl;
                test.result(Test::fail);
            }

            // Num of navigation pairs set in navigation config (above)
            //
            if ((nav_pairs.size() / sizeof(Navigation_pair)) > nav_cfg.max_peaks_per_azimuth()) {
                stdout_log << "Too many navigation pairs. " << endl;
                test.result(Test::fail);
            }
        }


        test.result(Test::pass);
    }
    
    // -----------------------------------------------------------------------------------------------------------------
    // calibrate_accelerometer      
    // start_accelerometer          
    // stop_accelerometer           
    // accelerometer_data

    void on_accelerometer_data(Message&, Test& test)
    {
        test.result(Test::pass);
    }           
    
    // -----------------------------------------------------------------------------------------------------------------
    // set_nav_buffer_config        
    // set_nav_bin_operation        
    // navigation_alarm_data

    void on_nav_alarm_data(Message& msg, Test& test)
    {
        auto nav_alarms     = msg.view_as<Navigation_alarm_data>();
        auto alarm_states   = nav_alarms->to_vector();

        if (alarm_states.size() == 6) test.result(Test::pass);
        else                          test.result(Test::fail);
    }                 
    
    // -----------------------------------------------------------------------------------------------------------------
    // nav_radar_reset              
    // nav_radar_halt               
    
    // -----------------------------------------------------------------------------------------------------------------
    // nav_area_rules_request
    // nav_area_rules

    namespace Navigation_rules {
        using namespace Navigation;

        Header header { };
        Rule   rule   { };
        std::array<Point, 4> area {
            Point { 0.0f, 0.0f },
            Point { 100.0f, 0.0f },
            Point { 100.0f, 100.0f },
            Point { 0.0f, 100.0f }
        };
    }

    void do_nav_area_rules(Message& msg, Test& test)
    {
        using namespace Navigation_rules;

        Message_buffer buffer { };

        header.rule_count(1);
        buffer << header;

        rule.id(1);
        rule.enabled(true);
        rule.invert_break_logic(false);
        rule.threshold_delta(1.0);
        rule.break_allowance(10);
        rule.allowance_curve_decrement(2);
        rule.point_count(area.size());
        buffer << rule;

        for (auto& pt : area) {
            buffer << pt;
        }

        msg.append(std::move(buffer));

        test.result(Test::pass);
    }


    void on_nav_area_rules(Message& msg, Test& test)
    {
        using namespace Navtech::Navigation;

        auto area_rules = msg.view_as<Navigation_area_rules>();
        auto data       = area_rules->to_vector();

        if (data.size() == 0) {
            test.result(Test::fail);
            return;
        }
		
        auto header = Header::overlay_at(data.data());
        
        if (header->rule_count() != 6) {
            test.result(Test::fail);
            return;
        }
		
        // The stream_iterator always points to the current rule.
        // 
        auto stream_iterator = header->end();

        // This the rule updated by do_nav_area_rules()
        //
        auto current_rule = Rule::overlay_at(stream_iterator);

        if (current_rule->id() != Navigation_rules::rule.id())                                               { test.result(Test::fail); return; }
        if (current_rule->enabled() != Navigation_rules::rule.enabled())                                     { test.result(Test::fail); return; }
        if (current_rule->threshold_delta() != Navigation_rules::rule.threshold_delta())                     { test.result(Test::fail); return; }
        if (current_rule->allowance_curve_decrement() != Navigation_rules::rule.allowance_curve_decrement()) { test.result(Test::fail); return; }
        if (current_rule->break_allowance() != Navigation_rules::rule.break_allowance())                     { test.result(Test::fail); return; }

        stdout_log << std::boolalpha;
        stdout_log << "Rule ID:             " << current_rule->id() << endl;
        stdout_log << "Enabled:             " << current_rule->enabled() << endl;
        stdout_log << "Threshold delta:     " << current_rule->threshold_delta() << endl;
        stdout_log << "Allowance curve dec: " << current_rule->allowance_curve_decrement() << endl;
        stdout_log << "Break allowance:     " << current_rule->break_allowance() << endl;

        auto [point_array, num_points] = current_rule->points();

        // Can't have an area with less than three points!
        //
        if (num_points < 3) test.result(Test::fail);

        for (std::size_t i { 0 }; i < num_points; ++i) {
            if (point_array[i] != Navigation_rules::area[i]) {
                test.result(Test::fail);
                return;
            }
        }

        test.result(Test::pass);
    }


    void on_time_server_status(Message& msg, Test& test)
    {
        auto status = msg.view_as<Time_server_status>();

        if (!status->ntp_enabled()) { test.result(Test::fail); return; }
        if (!status->ntp_sync())    { test.result(Test::fail); return; }
        
        if (Time::abs_diff(Time::Real_time::now(), status->now()) > 1_sec) { test.result(Test::fail); return; }

        test.result(Test::pass);
    }

} // namespace Navtech::Networking::Colossus_protocol