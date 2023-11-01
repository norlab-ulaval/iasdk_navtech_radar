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

#ifndef DATAGRAM_SERVER_H
#define DATAGRAM_SERVER_H

#include "Datagram_server_traits.h"
#include "Connection_manager.h"
#include "Active.h"
#include "Endpoint.h"
#include "Log.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

namespace Navtech::Networking {

    template <Protocol protocol, TLS::Policy tls_policy = TLS::Policy::none>
    class Datagram_server : public Utility::Active {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unweildy) these parameters are looked up from the Connection_traits
        // class, using the appropriate combination of protocol and tls_policy
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol>;
        using Server_traits     = Datagram_server_traits<protocol, tls_policy>;
        using Connection_Ty     = typename Server_traits::Connection;
        using Socket_Ty         = typename Server_traits::Socket;
        using Message_Ty        = typename Server_traits::Message;
        using ID_Ty             = typename Server_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;


        Datagram_server(Dispatcher_Ty& event_dispatcher);
        Datagram_server(const Endpoint& local_endpt, Dispatcher_Ty& event_dispatcher);
        Datagram_server(const Endpoint& local_endpt, const Endpoint& remote_endpt, Dispatcher_Ty& event_dispatcher);
        Datagram_server(const Endpoint& local_endpt, const Endpoint& remote_endpt, std::uint8_t TTL, Dispatcher_Ty& event_dispatcher);

        void bind_to(const Endpoint& local_endpt);
        void send_to(const Endpoint& remote_endpt);

        void send(const Message_Ty& msg);
        void send(Message_Ty&& msg);

    protected:

    private:
        // External associations
        //
        association_to<Dispatcher_Ty> dispatcher;

        // NOTE - declaration order is important!
        //        connection is dependent on an initialised socket
        //
        ID_Ty         id;
        Socket_Ty     socket;
        Connection_Ty connection;

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
    };



    template <Protocol protocol, TLS::Policy tls_policy>
    Datagram_server<protocol, tls_policy>::Datagram_server(Datagram_server<protocol, tls_policy>::Dispatcher_Ty& event_dispatcher) :
        Active          { "Datagram Server" },
        dispatcher      { associate_with(event_dispatcher) },
        id              { 1 },
        socket          {  },
        connection      { id, std::move(socket), event_dispatcher }
    {
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    Datagram_server<protocol, tls_policy>::Datagram_server(
        const Endpoint&                                     local_endpt,
        Datagram_server<protocol, tls_policy>::Dispatcher_Ty&  event_dispatcher
    ) :
        Active          { "Datagram Server" },
        dispatcher      { associate_with(event_dispatcher) },
        id              { 1 },
        socket          { local_endpt },
        connection      { id, std::move(socket), event_dispatcher }
    {
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    Datagram_server<protocol, tls_policy>::Datagram_server(
        const Endpoint&                                     local_endpt, 
        const Endpoint&                                     remote_endpt,
        Datagram_server<protocol, tls_policy>::Dispatcher_Ty&  event_dispatcher) :
        Active          { "Datagram Server" },
        dispatcher      { associate_with(event_dispatcher) },
        id              { 1 },
        socket          { local_endpt, remote_endpt },
        connection      { id, std::move(socket), event_dispatcher }
    {
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    Datagram_server<protocol, tls_policy>::Datagram_server(
        const Endpoint&                                     local_endpt, 
        const Endpoint&                                     remote_endpt,
        std::uint8_t                                        TTL,
        Datagram_server<protocol, tls_policy>::Dispatcher_Ty&  event_dispatcher) :
        Active          { "Datagram Server" },
        dispatcher      { associate_with(event_dispatcher) },
        id              { 1 },
        socket          { local_endpt, remote_endpt, TTL },
        connection      { id, std::move(socket), event_dispatcher }
    {
    }

    template <Protocol protocol, TLS::Policy tls_policy>
    void Datagram_server<protocol, tls_policy>::bind_to(const Endpoint& local_endpt)
    {
        socket.bind_to(local_endpt);
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Datagram_server<protocol, tls_policy>::send_to(const Endpoint& remote_endpt)
    {
        connection.remote_endpoint(remote_endpt);
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Datagram_server<protocol, tls_policy>::on_start()
    {
        error_handler.when_notified_invoke(
            [this](ID_Ty connection_id) 
            { 
                async_call(&Datagram_server::on_error, this, connection_id); 
            }
        );
    
        dispatcher->template attach_to<Event_traits::Connection_error>(error_handler);

        enabled = true;
        connection.open();
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Datagram_server<protocol, tls_policy>::on_stop()
    {
        enabled = false;
        dispatcher->template detach_from<Event_traits::Connection_error>(error_handler);
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Datagram_server<protocol, tls_policy>::on_error(ID_Ty connection_id)
    {
        if (connection_id != id) return; // Error is not for us.

        enabled = false;
        connection.close();
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Datagram_server<protocol, tls_policy>::send(const Message_Ty& msg)
    {
        connection.send(msg);
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Datagram_server<protocol, tls_policy>::send(Message_Ty&& msg)
    {
        connection.send(std::move(msg));
    }

} // namespace Navtech::Networking


#endif // DATAGRAM_SERVER_H