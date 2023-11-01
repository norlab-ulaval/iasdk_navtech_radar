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

#ifndef EVENT_HANDLER_H
#define EVENT_HANDLER_H

#include <map>
#include <functional>
#include <any>
#include <memory>
#include <vector>

// ==============================================================================================
// The Event_dispatcher library is a variation on the Command Pattern.
// Clients may register for ('attach_to') events with handler callbacks.
// Sources-of-events may notify an event, triggering any registered handlers
// for that event to be invoked.
//
// Events are defined as enumerated types.
//
// This library extends the basic Command pattern by allowing each event to have its own
// (unique) data type associated with it.  That is, when the event is notified an object - 
// specific to that event - can be provided.  This object is passed to each registered
// handler.  Event handlers must provide a function that takes a parameter of the correct type.
//
// Event parameters are specified through a template traits class.  Examples of possible traits
// are shown below:
//
// ------------------------------------------------------------------------------------
// An enumeration representing a set of (related) events.
//
// For this (set of) events, all events have the same data
// associated with them.  The type of data associated with the event 
// is defined in the Message_traits class.
// Note, the trait class type alias *must* be named 'Parameter', as this 
// will be looked for by the Dispatcher class.
//
// enum class Message { config, health, fft };
//
// template <Message> struct Message_traits { using Parameter = std::string; };
//
// ------------------------------------------------------------------------------------
// Each Event has a different type of data associated with it. This
// is done by specialising the trait class for each particular enum
//
// enum class Event { go, stop, wait, finish };
//
// template <Event> struct Event_traits { };
// template <> struct Event_traits<Event::go>     { using Parameter = int; }; 
// template <> struct Event_traits<Event::stop>   { using Parameter = std::string; }; 
// template <> struct Event_traits<Event::wait>   { using Parameter = void; }; 
// template <> struct Event_traits<Event::finish> { using Parameter = ADT; }; 
//
// ------------------------------------------------------------------------------------
// A plain enum can be used for events.
// In this example, all Signals use int as their data type
// *except* stop, which has std::string as its associated type.
//
// enum Signal { start, stop, reset, shudown };
//
// template <Signal> struct Signal_traits { using Parameter = int; };
// template <> struct Signal_traits<stop> { using Parameter = std::string; };


namespace Navtech::Utility {

    // ------------------------------------------------------------------------------------
    // The Event_handler class wraps a callable type (via an internal std::function) that 
    // is the handler function for the specific event.
    // An Event_handler's template parameter is the type of the argument provided when the
    // event is notified.  The template parameter must match the type specified by the Event
    // (enum) trait class.
    // Using an Event_handler class allows clients to attach to, and detach from, events by
    // passing the Event_handler object as an argument.
    // The Event_handler's call operator only supports a single argument passed by 
    // reference-to-const.  The attached function must make its own copy of the argument if
    // it wants to modify it; to prevent changing the argument for other Event_handlers that
    // may also be called.
    //
    // General case
    //
    template <typename T = void>
    class Event_handler {
    public:
        Event_handler() = default;

        template <typename Callable_Ty>
        explicit Event_handler(Callable_Ty&& handler_fn) : 
            fn { std::forward<Callable_Ty>(handler_fn) }
        {
        }

        void operator()(const T& arg)
        {
            if (fn) fn(arg);
        }

        template <typename Callable_Ty>
        void when_notified_invoke(Callable_Ty&& handler_fn)
        {
            fn = std::forward<Callable_Ty>(handler_fn);
        }

        template <typename Class_Ty>
        void when_notified_invoke(void(Class_Ty::* mem_fn_ptr)(const T&), Class_Ty* const obj_ptr)
        {
            fn = std::bind(mem_fn_ptr, obj_ptr, std::placeholders::_1);
        }

        template <typename Callable_Ty>
        Event_handler& operator=(Callable_Ty&& handler_fn)
        {
            fn = std::forward<Callable_Ty>(handler_fn);
            return *this;
        }

        operator bool() const
        {
            return (fn != nullptr);
        }

    private:
        std::function <void(const T&)> fn { };
    };


    // Specialisation for handlers with no argument
    //
    template <>
    class Event_handler<void> {
    public:
        Event_handler() = default;

        template <typename Callable_Ty>
        Event_handler(Callable_Ty&& handler_fn) : 
            fn { std::forward<Callable_Ty>(handler_fn) }
        {
        }

        void operator()()
        {
            if (fn) {
                fn();
            }
        }

        template <typename Callable_Ty>
        void when_notified_invoke(Callable_Ty&& handler_fn)
        {
            fn = std::forward<Callable_Ty>(handler_fn);
        }

        template <typename Class_Ty>
        void when_notified_invoke(void(Class_Ty::* mem_fn_ptr)(), Class_Ty* const obj_ptr)
        {
            fn = std::bind(mem_fn_ptr, obj_ptr);
        }

        template <typename Callable_Ty>
        Event_handler& operator=(Callable_Ty&& handler_fn)
        {
            fn = std::forward<Callable_Ty>(handler_fn);
            return *this;
        }

        operator bool() const
        {
            return (fn != nullptr);
        }

    private:
        std::function <void()> fn { };
    };

    // Member functions are commonly used as event handler functions.  Since the
    // signature is common, this macro can simplify user code.
    //
    #define mem_fn_handler(fn) std::bind(fn, this, std::placeholders::_1)

    // ------------------------------------------------------------------------------------
    // The Event_map class is the set of Event handlers for an event.
    // The class holds the data (of template parameter type T) and a vector of Event_handlers.  
    // When the Event_handler's data is updated (with notify()) the class iterates through its 
    // Event_handlers, passing the data to each in turn.
    //
    template <typename T>
    class Event_map {
    public:
        using Handler   = Event_handler<T>;
        using Container = std::vector<Handler*>;

        void attach(Handler& handler)
        {
            event_handlers.push_back(&handler);
        }

        void detach(Handler& handler)
        {
            using std::begin;
            using std::end;
            using std::remove_if;

            event_handlers.erase(
                remove_if(
                    begin(event_handlers),
                    end  (event_handlers),
                    [&handler](Handler* h) { return h == &handler; }
                ),
                end(event_handlers)
            );
        }

        void notify(const T& value)
        {
            for (auto& handler : event_handlers) (*handler)(value);
        }

        void notify(T&& value)
        {
            using std::move;
            
            // Move into local copy, to maintain move
            // semantics for clients
            //
            T data { move(value) };
            for (auto& handler : event_handlers) (*handler)(data);
        }

    private:
        Container event_handlers { };
    };


    // Template specialisation for events with no data associated
    //
    template <>
    class Event_map<void> {
    public:
        using Handler   = Event_handler<void>;
        using Container = std::vector<Handler*>;

        void attach(Handler& handler)
        {
            event_handlers.push_back(&handler);
        }

        void detach(Handler& handler)
        {
            using std::begin;
            using std::end;
            using std::remove_if;

            event_handlers.erase(
                remove_if(
                    begin(event_handlers),
                    end  (event_handlers),
                    [&handler](Handler* h) { return h == &handler; }
                ),
                end(event_handlers)
            );
        }

        void notify()
        {
            for (auto& handler : event_handlers) (*handler)();
        }

    private:
        Container event_handlers { };
    };


    // ------------------------------------------------------------------------------------
    // The Dispatcher binds events against event handlers.
    // In the STL a container can only hold one type of object.  In this problem, each event 
    // can have a different type of data (and event handlers) associated with it.  The 
    // solution is to store the Event_handler in type-erased form, using std::any.
    //
    // The attach_to() and notify() functions are functions templates. The client supplies a 
    // template parameter, in the form of an event enum, and this is used to cast the std::any 
    // object back to an Event_handler of the correct type.  The cast is done indirectly via 
    // the Trait_Ty: Given an event type, the compiler looks up the associated data type using 
    // the event's associated trait class. This lookup is done at compile-time, not at run-time.
    //
    // To avoid introducing another level of template indirection (without adding any real benefit) 
    // the Dispatcher requires two template paramters - the event enum type, and its associated 
    // traits class
    //
    template <typename Event_Ty, template <Event_Ty> class Trait_Ty>
    class Dispatcher {
    public:
        template <Event_Ty e>
        void attach_to(Event_handler<typename Trait_Ty<e>::Parameter>& handler);

        template <Event_Ty e>
        void detach_from(Event_handler<typename Trait_Ty<e>::Parameter>& handler);

        template <Event_Ty e>
        void notify();

        template <Event_Ty e>
        void notify(typename Trait_Ty<e>::Parameter&& value);

        template <Event_Ty e>
        void notify(const typename Trait_Ty<e>::Parameter& value);

    private:
        std::map<Event_Ty, std::any> events { };

        static_assert(std::is_enum<Event_Ty>::value, "Dispatcher template parameter must be an enum type.");
    };


    template <typename Event_Ty, template <Event_Ty> class Trait_Ty>
    template <Event_Ty e>
    void Dispatcher<Event_Ty, Trait_Ty>::attach_to(Event_handler<typename Trait_Ty<e>::Parameter>& handler)
    {
        using std::any_cast;

        using Type = typename Trait_Ty<e>::Parameter;
        using Map  = Event_map<Type>;

        auto  emplace_result = events.emplace(e, Map { });
        auto& event_map      = any_cast<Map&>(emplace_result.first->second);
        event_map.attach(handler);
    }


    template <typename Event_Ty, template <Event_Ty> class Trait_Ty>
    template <Event_Ty e>
    void Dispatcher<Event_Ty, Trait_Ty>::detach_from(Event_handler<typename Trait_Ty<e>::Parameter>& handler)
    {
        using std::any_cast;

        using Type = typename Trait_Ty<e>::Parameter;
        using Map  = Event_map<Type>;

        auto  emplace_result = events.emplace(e, Map { });
        auto& event_map      = any_cast<Map&>(emplace_result.first->second);
        event_map.detach(handler);
    }


    template <typename Event_Ty, template <Event_Ty> class Trait_Ty>
    template <Event_Ty e>
    void Dispatcher<Event_Ty, Trait_Ty>::notify(typename Trait_Ty<e>::Parameter&& value)
    {
        using std::any_cast;
        using std::move;

        using Type = typename Trait_Ty<e>::Parameter;
        using Map  = Event_map<Type>;

        auto iter = events.find(e);
        if (iter != end(events)) {
            auto& event_map = any_cast<Map&>(iter->second);
            event_map.notify(move(value));
        }
    }


    template <typename Event_Ty, template <Event_Ty> class Trait_Ty>
    template <Event_Ty e>
    void Dispatcher<Event_Ty, Trait_Ty>::notify(const typename Trait_Ty<e>::Parameter& value)
    {
        using std::any_cast;

        using Type = typename Trait_Ty<e>::Parameter;
        using Map  = Event_map<Type>;

        auto iter = events.find(e);
        if (iter != end(events)) {
            auto& event_map = any_cast<Map&>(iter->second);
            event_map.notify(value);
        }
    }


    template <typename Event_Ty, template <Event_Ty> class Trait_Ty>
    template <Event_Ty e>
    void Dispatcher<Event_Ty, Trait_Ty>::notify()
    {
        using std::any_cast;

        using Type = typename Trait_Ty<e>::Parameter;
        using Map  = Event_map<Type>;

        auto iter = events.find(e);
        if (iter != end(events)) {
            auto& event_map = any_cast<Map&>(iter->second);
            event_map.notify();
        }
    }

} // namespace Navtech::Utility


#endif // EVENT_HANDLER_H