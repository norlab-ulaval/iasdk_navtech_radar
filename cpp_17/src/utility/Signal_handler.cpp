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

#include "Signal_handler.h"

namespace Navtech::Utility {

    Signal_handler* Signal_handler::self { };


    Signal_handler::Signal_handler()
    {
        self = this;
    }


    void Signal_handler::register_handler(std::int32_t signal, Signal_handler::Action callback)
    {
        if (callback == nullptr) return;

        signal_actions[signal] = std::move(callback);
        set_signal_handler(signal);
    }


    void Signal_handler::unregister_handler(std::int32_t signal) 
    { 
        signal_actions.erase(signal); 
    }


#ifdef __linux__
    void Signal_handler::signal_action(std::int32_t signal, siginfo_t* info, void*)
    {
        if (!self) return;

        if (self->signal_actions.count(signal) != 0) {
            self->signal_actions[signal](signal, info->si_int);
        }
    }


    void Signal_handler::set_signal_handler(int sig)
    {
        struct sigaction new_action { };

        new_action.sa_sigaction = &Signal_handler::signal_action;
        sigemptyset(&new_action.sa_mask);
        new_action.sa_flags = SA_SIGINFO | SA_RESTART;

        struct sigaction old_action { };
        
        sigaction(sig, nullptr, &old_action);

        if (old_action.sa_handler != SIG_IGN) sigaction(sig, &new_action, nullptr);
    }

#elif _WIN32
    void Signal_handler::signal_action(std::int32_t signal)
    {
        if (!self) return;

        if (self->signal_actions.count(signal) != 0) {
            self->signal_actions[signal](signal, 0);
        }
    }


    void Signal_handler::set_signal_handler(int sig)
    {
        signal(sig, &&Signal_handler::signal_action);
    }

#endif


} // namespace Navtech::Utility