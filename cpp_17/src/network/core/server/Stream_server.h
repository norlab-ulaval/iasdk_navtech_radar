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

#ifndef STREAM_SERVER_H
#define STREAM_SERVER_H

#include "Stream_server_traits.h"
#include "Event_traits.h"
#include "Acceptor.h"
#include "Connection_manager.h"
#include "Active.h"
#include "Endpoint.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

namespace Navtech::Networking {

    template <Protocol protocol, TLS::Policy tls_policy = TLS::Policy::none>
    class Stream_server : public Utility::Active {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unweildy) these parameters are looked up from the Connection_traits
        // class, using the appropriate combination of protocol and tls_policy
        //
        using Event_traits          = Navtech::Networking::Event_traits<protocol>;
        using Server_traits         = Stream_server_traits<protocol, tls_policy>;
        using Socket_Ty             = typename Server_traits::Socket;
        using Socket_ptr_Ty         = typename Server_traits::Socket_ptr;
        using Message_Ty            = typename Server_traits::Message;
        using Addressed_message_Ty  = typename Server_traits::Addressed_message;
        using ID_Ty                 = typename Server_traits::ID;
        using Dispatcher_Ty         = typename Event_traits::Dispatcher;

        Stream_server(Dispatcher_Ty& event_dispatcher);
        Stream_server(Port port, Dispatcher_Ty& event_dispatcher);

        void bind_to(Port port);

        void send(const Addressed_message_Ty& msg);
        void send(Addressed_message_Ty&& msg);

    protected:
        friend class Acceptor<protocol, tls_policy>;

        // Asynchronous interface
        //
        void start_connection(const Socket_ptr_Ty& socket);
        void start_connection_impl(const Socket_ptr_Ty& socket);

    private:
        // External associations
        //
        association_to<Dispatcher_Ty> dispatcher;

        Acceptor<protocol, tls_policy>           acceptor       { };
        Connection_manager<protocol, tls_policy> connection_mgr { };

        // Active object overrides
        //
        void on_start() override;
        void on_stop()  override;

        Utility::Event_handler<ID_Ty> error_handler { };
        void on_error(ID_Ty connection_id);

        // Internal state
        //
        bool enabled { false };

        // Implementation
        //
        void send_to(ID_Ty id, const Message_Ty& msg);
        void send_to(ID_Ty id, Message_Ty&& msg);
        void send_to_all(const Message_Ty& msg);
        void send_to_all(Message_Ty&& msg);     
    };



    template <Protocol protocol, TLS::Policy tls_policy>
    Stream_server<protocol, tls_policy>::Stream_server(Stream_server<protocol, tls_policy>::Dispatcher_Ty& event_dispatcher) :
        Active      { "Stream Server" },
        dispatcher  { associate_with(event_dispatcher) },
        acceptor    { *this }
    {
    }

    template <Protocol protocol, TLS::Policy tls_policy>
    Stream_server<protocol, tls_policy>::Stream_server(
        Port                                                port,
        Stream_server<protocol, tls_policy>::Dispatcher_Ty&    event_dispatcher
    ) :
        Active      { "Stream Server" },
        dispatcher  { associate_with(event_dispatcher) },
        acceptor    { *this, port }
    {
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::bind_to(Port port)
    {
        acceptor.bind_to(port);
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::on_start()
    {
        error_handler.when_notified_invoke(
            [this](ID_Ty connection_id) 
            { 
                async_call(&Stream_server::on_error, this, connection_id); 
            }
        );
        
        dispatcher->template attach_to<Event_traits::Connection_error>(error_handler);

        acceptor.start();
        enabled = true;
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::on_stop()
    {
        enabled = false;

        dispatcher->template detach_from<Event_traits::Connection_error>(error_handler);

        acceptor.stop();
        acceptor.join();
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::on_error(ID_Ty connection_id)
    {
        enabled = false;
        connection_mgr.remove(connection_id);
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::start_connection(const Socket_ptr_Ty& socket_ptr)
    {
        async_call(&Stream_server<protocol, tls_policy>::start_connection_impl, this, std::move(socket_ptr));
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::start_connection_impl(const Socket_ptr_Ty& socket_ptr)
    {
        connection_mgr.create_connection(std::move(*socket_ptr));
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::send(const Addressed_message_Ty& msg)
    {
        // ID_Ty zero is the 'broadcast' address
        //
        if (msg.id == 0) send_to_all(msg.message);
        else             send_to(msg.id, msg.message);
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::send(Addressed_message_Ty&& msg)
    {
        // Message_Ty will be copied
        if (msg.id == 0) send_to_all(std::move(msg.message));
        else             send_to(msg.id, std::move(msg.message));
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::send_to(ID_Ty id, const Message_Ty& msg)
    {
        auto connection = connection_mgr.connection(id);

        if (connection) connection->send(msg);
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::send_to(ID_Ty id, Message_Ty&& msg)
    {
        auto connection = connection_mgr.connection(id);

        if (connection) 
        connection->send(std::move(msg));
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::send_to_all(const Message_Ty& msg)
    {
        auto connections = connection_mgr.all_connections();

        for (auto connection : connections) {
            if (connection) connection->send(msg);
        }
    } 


    template <Protocol protocol, TLS::Policy tls_policy>
    void Stream_server<protocol, tls_policy>::send_to_all(Message_Ty&& msg)
    {
        // In the special case where there is only one client
        // the message will be moved; otherwise it must be copied
        //
        auto connections = connection_mgr.all_connections();

        if (connections.size() == 1) {
            connections[0]->send(std::move(msg));
        }
        else {
            for (auto connection : connections) {
                if (connection) connection->send(msg);
            }
        }
    } 



    // Aliases for commonly-used clients
    //
    using Colossus_server = Stream_server<Protocol::colossus, TLS::Policy::none>;

} // namespace Navtech::Networking

#endif // STREAM_SERVER_H