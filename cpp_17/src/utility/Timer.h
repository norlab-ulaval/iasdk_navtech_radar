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

#ifndef TIMER_H
#define TIMER_H

#include "Time_utils.h"
#include "Active.h"

namespace Navtech::Utility {

    using namespace Navtech::Time;


    class Timer : public Active {
    public:
        enum Mode { one_shot, repeating };

        Timer(const Time::Duration& timeout, Mode m = repeating);
        Timer(const Time::Duration& timeout, std::function<void()> on_timeout, Mode m = repeating);

        void on_timeout(std::function<void()> fn);      

    private:
        using Tick = std::uint32_t;

        static constexpr Time::Duration resolution { 10_msec };
        
        Tick                    timeout         { };
        Tick                    tick            { };
        std::function<void()>   callback        { };
        bool                    is_repeating    { true };

        Active::Task_state run() override;
    };

} // namespace Navtech::Utility

#endif // TIMER_H