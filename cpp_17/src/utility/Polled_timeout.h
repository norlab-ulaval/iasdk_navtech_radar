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

#ifndef POLLED_TIMEOUT_H
#define POLLED_TIMEOUT_H

#include "Time_utils.h"

namespace Navtech::Time {

    class Polled_timeout {
    public:
        enum Type { one_shot, resetting };

        Polled_timeout(Duration timeout_period, Type timer_type = resetting) : 
            period      { timeout_period },
            is_one_shot { timer_type == one_shot }
        {
        }

        void start()
        {
            is_running    = true;
            last_reset = Monotonic::Clock::now();
        }

        void stop()
        {
            is_running = false;
        }

        bool running() const
        {
            return is_running;
        }

        bool expired()
        {
            auto now        = Monotonic::Clock::now();
            auto delta_t    = to_nearest_millisecond(now - last_reset);
            auto timed_out  = delta_t >= period;
            auto is_expired = is_running && timed_out;
            
            if (is_expired && is_one_shot) stop();

            return is_expired;
        }

        void reset()
        {
            last_reset = Monotonic::Clock::now();
        }

    private:
        Duration               period      { };
        Monotonic::Observation last_reset  { };
        bool                   is_running  { };
        bool                   is_one_shot { };
    };

} // namespace Navtech::Time

#endif // POLLED_TIMEOUT_H