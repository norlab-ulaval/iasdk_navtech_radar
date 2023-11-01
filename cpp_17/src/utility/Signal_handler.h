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

#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

#include <cstdint>
#include <functional>
#include <map>

#include <signal.h>

namespace Navtech::Utility {

    class Signal_handler {
    public:
        using Action = std::function<void(std::int32_t signal, std::int32_t info)>;

        Signal_handler();
        
        void register_handler(std::int32_t signal, Action = nullptr);
        void unregister_handler(std::int32_t signal);

    private:
        std::map<std::int32_t, Action> signal_actions;

#ifdef __linux__
        using Signal_callback = void (std::int32_t signal, siginfo_t* info, void* unused);
#elif _WIN32
        using Signal_callback = void (std::int32_t signal, siginfo_t* info, void* unused);
#endif
        void set_signal_handler(int sig);

        static Signal_callback signal_action; 
        static Signal_handler* self;
    };

} // namespace Navtech

#endif // SIGNAL_HANDLER_H
