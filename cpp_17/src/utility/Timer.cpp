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

#include "Timer.h"

namespace Navtech::Utility {

    Timer::Timer(const Time::Duration& time_out, Timer::Mode m) :
        timeout         { static_cast<Tick>(time_out / resolution) },
        is_repeating    { m == Timer::Mode::repeating }
    {
    }

    Timer::Timer(const Time::Duration& time_out, std::function<void()> on_timeout, Timer::Mode m) :
        timeout         { static_cast<Tick>(time_out / resolution) },
        callback        { std::move(on_timeout) },
        is_repeating    { m == Timer::Mode::repeating }
    {
    }

    
    void Timer::on_timeout(std::function<void()> fn)
    {
        callback = std::move(fn);
    }


    Active::Task_state Timer::run()
    {
        Task_state run_state { Task_state::not_finished };

        Time::Monotonic::sleep_for(resolution);
        ++tick;

        if (tick == timeout) {
            if (callback) callback();

            if (is_repeating) {
                tick = 0;
                run_state = Task_state::not_finished;
            }   
            else {
                run_state = Task_state::finished;
            }
        }

        return run_state;
    }

} // namespace Navtech::Utility