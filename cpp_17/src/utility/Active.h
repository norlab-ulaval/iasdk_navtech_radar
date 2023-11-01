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

#ifndef ACTIVE_H
#define ACTIVE_H

#include <thread>
#include <functional>
#include <string>
#include <string_view>

#include "Threadsafe_queue.h"

namespace Navtech::Utility {

    // ---------------------------------------------------------------------------------------------------
    // The Active class represents a thread-of-control that supports an asynchronous
    // messaging interface.
    // Derived classes can implement an asynchronous interface by implementing two
    // methods:
    // - a public function that represents the client call.  It may have parameters, but
    //   cannot return any value (that, by definition would make it a synchronous call!)
    //   The public function must call async_call(), providing the address of the 
    //   implementation function and the arguments for the call.
    //   NOTE: parameters are passed by copy - to pass a reference you must wrap the
    //   object in a std::ref when invoking async_call().
    //   NOTE: The public interface function runs in the context of the caller.  Therefore
    //   this function should *never* attempt to modify the state of the object.  Always use
    //   the implementation function to modify the object's state.
    // - The implementation function.  This function should have the same signature (parameters)
    //   as the public interface function.  All the behaviour of the call should be implemented
    //   here.
    //   The implementation function runs in the context of this thread, therefore it is
    //   safe to modify any of the object's state in this function.  The implementation function
    //   can be written as if it is synchronous code (which it is!)
    //
    // The default behaviour is to pend on the Active class's message queue.  If an aysnchronous 
    // call is received it is dispatched, then the class blocks again.  In this way an Active 
    // object behaves like a synchronous object - it does nothing unless one of its methods is invoked.
    //
    // A derived class may override the run() method to peform behaviour when *not* waiting for
    // asynchronous calls.
    // The default 'run policy' is to keep calling the run() function while it returns a value of
    // Task_state::not_finished.  The derived class's run() function does not have to implement any
    // kind of 'main loop' - the run policy provides that.
    // It is recommended that a derived class's run() method periodically checks the asynchronous
    // message queue.  This can be done with the try_async_dispatch() method.
    //
    // An Active class will not begin executing its behaviour (or responding to asynchronous messages)
    // until start() has been called.
    // An Active class can be asynchronously stopped by calling the stop() method.
    // Destroying an Active object will safely terminate its behaviour.  If a derived class has
    // implemented its own run() function, that will complete before the class goes out of scope.
    // (This is another reason why a run() function must never contain non-terminating loops!)
    //
    // Two overridable functions, on_start() and on_stop() are invoked as follows:
    // - on_start() is called before the internal thread is started.  This function is therefore
    //   useful to configure your active object prior to starting its thread-of-control
    // - on_stop() is invoked before the stop() asynchronous handling.  Any messages
    //   on the message queue will still be available.
    //
    class Active {
    public:
        enum class Task_state { not_finished, finished };

        Active() = default;
        Active(std::string_view name_str);
        virtual ~Active();

        void start();
        void stop();
        void join();

        virtual void on_start() { }
        virtual void on_stop()  { }

    protected:
        template <typename Callable_Ty, typename... Arg_Ty>
        void async_call(Callable_Ty&& callable, Arg_Ty... arg)
        {
            msg_queue.push(std::bind(callable, std::forward<Arg_Ty>(arg)...));
        }

        virtual void run_policy();
        virtual Task_state run();
        
        bool try_dispatch_async();

    private:
        std::thread t;

        using Impl_fn = std::function<void()>;
        using Queue   = Threadsafe_queue<Impl_fn>;
        
        Queue msg_queue { };
        bool created { false };
        bool running { false };
        std::string name { "unknown" };

        void stop_impl();
        void dispatch_async();
    };

} // namespace Navtech::Utility

#endif // ACTIVE_H