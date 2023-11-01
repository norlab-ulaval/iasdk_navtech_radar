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

#ifndef DISPATCHER_H
#define DISPATCHER_H

#include <unordered_map>

#include "Stream_client_traits.h"
#include "Event_traits.h"
#include "Active.h"
#include "pointer_types.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

namespace Navtech::Networking {
    
    // =================================================================================================================
    //
    // Message dispatcher handles all incoming messages.
    // The Message dispatcher runs in its own thread-of-control and invokes
    // message handling callbacks for incoming messages (if registered)
    // The Dispatcher registers for the 'recieved message' networking event. 
    // This posts the message onto the Message_dispatcher's internal queue 
    // (which it inherits from the Active base class).  This means all message 
    // handling callbacks will execute in the context of the Message_dispatcher; 
    // and not in the caller's context.
    //
    template <Protocol protocol, typename Sender_Ty>
    class Message_dispatcher : public Utility::Active {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unweildy) these parameters are looked up from the Networking::Traits
        // class, using the appropriate combination of protocol and tls_policy
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol>;
        using Protocol_traits   = Navtech::Networking::Protocol_traits<protocol>;
        using Message_Ty        = typename Protocol_traits::Message;
        using Type_Ty           = typename Protocol_traits::Type;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;
        using Handler           = std::function<void(Sender_Ty&, Message_Ty&)>;
        
        Message_dispatcher(Sender_Ty& msg_sender, Dispatcher_Ty& event_dispatcher);
        
        // Async interface
        //
        void attach_to(Type_Ty type, const Handler& handler)  { async_call(&Message_dispatcher::attach_impl, this, type, handler); }
        void detach_from(Type_Ty type)                        { async_call(&Message_dispatcher::detach_impl, this, type); }

    protected:
        void on_stop() override;

    private:
        // External associations
        //
        association_to<Sender_Ty>     sender { };
        association_to<Dispatcher_Ty> dispatcher { };

        Utility::Event_handler<Message_Ty> receive_handler { };

        std::unordered_map<Type_Ty, Handler> message_handlers { };

        void dispatch(Message_Ty& msg);
        void attach_impl(Type_Ty type, const Handler& handler);
        void detach_impl(Type_Ty type);                      
    };


    using Utility::Active;

    template <Protocol protocol, typename Sender_Ty>
    Message_dispatcher<protocol, Sender_Ty>::Message_dispatcher(
        Sender_Ty&                                              msg_sender, 
        Message_dispatcher<protocol, Sender_Ty>::Dispatcher_Ty& event_dispatcher
    ) : 
        Utility::Active { "Message dispatcher" },
        sender          { associate_with(msg_sender) },
        dispatcher      { associate_with(event_dispatcher) }
    {
        receive_handler.when_notified_invoke(
            [this](const Message_Ty& msg) 
            {
                async_call(&Message_dispatcher::dispatch, this, msg); 
            }
        );
        
        dispatcher->template attach_to<Event_traits::Received_message>(receive_handler);
    }


    template <Protocol protocol, typename Sender_Ty>
    void Message_dispatcher<protocol, Sender_Ty>::attach_impl(Type_Ty type, const Handler& handler)
    {
        message_handlers.try_emplace(type, handler);
    }


    template <Protocol protocol, typename Sender_Ty>
    void Message_dispatcher<protocol, Sender_Ty>::detach_impl(Type_Ty type)
    {
        message_handlers.erase(type);
    }


    template <Protocol protocol, typename Sender_Ty>
    void Message_dispatcher<protocol, Sender_Ty>::dispatch(Message_Ty& msg)
    {
        if (message_handlers.count(Protocol_traits::type(msg)) == 1) {
            message_handlers[Protocol_traits::type(msg)](*sender, msg);
        }
        else {
            stdout_log << Logging_level::debug 
                       << "No handler for message type [" << static_cast<int>(Protocol_traits::type(msg)) << "]" 
                       << endl;
        }
    }


    template <Protocol protocol, typename Sender_Ty>
    void Message_dispatcher<protocol, Sender_Ty>::on_stop()
    {
        dispatcher->template detach_from<Event_traits::Received_message>(receive_handler);
    }


    // =================================================================================================================
    //
    // Addressed Message dispatcher handles messages for a specified Connection.
    // Any messages not the the Connection are ignored.
    //
    template <Protocol protocol, typename Sender_Ty>
    class Addressed_message_dispatcher : public Utility::Active {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unweildy) these parameters are looked up from the Networking::Traits
        // class, using the appropriate combination of protocol and tls_policy
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol>;
        using Protocol_traits   = Navtech::Networking::Protocol_traits<protocol>;
        using Message_Ty        = typename Protocol_traits::Message;
        using Type_Ty           = typename Protocol_traits::Type;
        using ID_Ty             = typename Protocol_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;
        using Handler           = std::function<void(Sender_Ty&, Message_Ty&)>;
        
        Addressed_message_dispatcher(ID_Ty listen_for, Sender_Ty& msg_sender, Dispatcher_Ty& event_dispatcher);
        
        // Async interface
        //
        void attach_to(Type_Ty type, const Handler& handler)  { async_call(&Addressed_message_dispatcher::attach_impl, this, type, handler); }
        void detach_from(Type_Ty type)                        { async_call(&Addressed_message_dispatcher::detach_impl, this, type); }

    protected:
        void on_stop() override;

    private:
        association_to<Sender_Ty>      sender { };
        association_to<Dispatcher_Ty>  dispatcher { };
        ID_Ty id { };

        Utility::Event_handler<Message_Ty> receive_handler { };

        std::unordered_map<Type_Ty, Handler> message_handlers { };

        void dispatch(Message_Ty& msg);
        void attach_impl(Type_Ty type, const Handler& handler);
        void detach_impl(Type_Ty type);                      
    };


    using Utility::Active;

    template <Protocol protocol, typename Sender_Ty>
    Addressed_message_dispatcher<protocol, Sender_Ty>::Addressed_message_dispatcher(
        Addressed_message_dispatcher<protocol, Sender_Ty>::ID_Ty          listen_for, 
        Sender_Ty&                                                        msg_sender,
        Addressed_message_dispatcher<protocol, Sender_Ty>::Dispatcher_Ty& event_dispatcher
    ) : 
        Utility::Active { "Addressed message dispatcher" },
        sender          { associate_with(msg_sender) },
        dispatcher      { associate_with(event_dispatcher) },
        id              { listen_for }
    {
        receive_handler.when_notified_invoke(
            [this](const Message_Ty& msg) 
            {
                if (msg.client_id() == id) {
                    async_call(&Addressed_message_dispatcher::dispatch, this, msg);
                }
            }
        );
      
        dispatcher->template attach_to<Event_traits::Received_message>(receive_handler);
    }


    template <Protocol protocol, typename Sender_Ty>
    void Addressed_message_dispatcher<protocol, Sender_Ty>::attach_impl(Type_Ty type, const Handler& handler)
    {
        message_handlers.try_emplace(type, handler);
    }


    template <Protocol protocol, typename Sender_Ty>
    void Addressed_message_dispatcher<protocol, Sender_Ty>::detach_impl(Type_Ty type)
    {
        message_handlers.erase(type);
    }


    template <Protocol protocol, typename Sender_Ty>
    void Addressed_message_dispatcher<protocol, Sender_Ty>::dispatch(Message_Ty& msg)
    {
        if (message_handlers.count(Protocol_traits::type(msg)) == 1) {
            message_handlers[Protocol_traits::type(msg)](*sender, msg);
        }
        else {
            stdout_log << Logging_level::debug 
                       << "No handler for message type [" << static_cast<int>(Protocol_traits::type(msg)) << "]" 
                       << endl;
        }
    }


    template <Protocol protocol, typename Sender_Ty>
    void Addressed_message_dispatcher<protocol, Sender_Ty>::on_stop()
    {
        dispatcher->template detach_from<Event_traits::Received_message>(receive_handler);
    }

} // namespace Navtech::Networking

#endif // DISPATCHER_H