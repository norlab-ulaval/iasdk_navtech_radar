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

#include "Active.h"
#include "Log.h"

using Navtech::Utility::stdout_log;

namespace Navtech::Utility {

    Active::Active(std::string_view name_str) :
        name { name_str }
    {
    }


    Active::~Active()
    {
        if (running) stop_impl();
        join();
    }


    void Active::start()
    {
        if (created) return;
        
        stdout_log << Logging_level::debug << "Starting active object [" << name << "]" << endl;

        on_start();
        running = true;
        t = std::thread { &Active::run_policy, this };

        auto handle = t.native_handle();
#ifdef __linux
        pthread_setname_np(handle, name.c_str());
#elif _WIN32
        SetThreadDescription(handle, name.c_str());
#endif

        created = true;
    }


    void Active::stop()
    {
        stdout_log << Logging_level::debug << "Async stop sent to active object [" << name << "]" << endl;

        on_stop();
        async_call(&Active::stop_impl, this);
    }


    void Active::join()
    {
        if (!created) return;
        if (t.joinable()) t.join();
    }


    void Active::stop_impl()
    {
        if (!running) return;
        running = false;
        msg_queue.clear();
        
        stdout_log << Logging_level::debug << "Stopped active object [" << name << "]" << endl;
    }


    void Active::run_policy()
    {
        Task_state state { };

        do {
            state = run();    
        } while (running && state != Task_state::finished);
    }
    
    
    Active::Task_state Active::run()
    {
        dispatch_async();
        return Task_state::not_finished;
    }


    void Active::dispatch_async()
    {
        auto msg { msg_queue.pop() };
        if (msg.has_value()) msg.value()();
    }


    bool Active::try_dispatch_async()
    {
        if (msg_queue.empty()) return false;

        dispatch_async();
        return true;
    }

} // namespace Navtech::Utility