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

#ifndef BYTESTREAM_CONNECTION_H
#define BYTESTREAM_CONNECTION_H

#include <atomic>

#include "Connection_traits.h"
#include "Event_traits.h"
#include "Active.h"
#include "pointer_types.h"
#include "Log.h"
#include "Time_utils.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using namespace Navtech::Time;

namespace Navtech::Networking::Bytestream {

    // =================================================================================================================
    //
    template <Protocol protocol, Navtech::TLS::Policy tls_policy = TLS::Policy::none>
    class Sender {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unweildy) these parameters are looked up from the Connection_traits
        // class, using the appropriate combination of protocol and tls_policy
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol>;
        using Connection_traits = Navtech::Networking::Connection_traits<protocol, tls_policy>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Socket_Ty         = typename Connection_traits::Socket;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        Sender(Socket_Ty& sckt, ID_Ty id, Dispatcher_Ty& event_dispatcher);

        void start();
        void stop();

        void send(const Message_buffer& msg);
        void send(Message_buffer&& msg);
    
    private:
        // External service associations
        //
        association_to<Socket_Ty> socket;
        association_to<Dispatcher_Ty> dispatcher;

        // Operating state
        //
        bool enabled { false };

        // Async function implementation
        //
        void do_send(Message_buffer& msg);

        // Identity
        //
        ID_Ty id;
    };



    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    Sender<protocol, tls_policy>::Sender(
        Sender<protocol, tls_policy>::Socket_Ty& sckt, 
        ID_Ty identity, 
        Sender<protocol, tls_policy>::Dispatcher_Ty& event_dispatcher
    ) :
        socket      { associate_with(sckt) },
        dispatcher  { associate_with(event_dispatcher) },
        id          { identity }
    {
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Sender<protocol, tls_policy>::start()
    {
        enabled = true;
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Sender<protocol, tls_policy>::stop()
    {
        enabled = false;
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Sender<protocol, tls_policy>::send(const Message_buffer& msg)
    {
        if (!enabled) return;

        try {
            socket->send(msg);
        }
        catch (std::system_error& ex) {
            stdout_log << Logging_level::debug << "Sender - failure to send: " << ex.what() << endl;
        
            dispatcher->template notify<Event_traits::Tx_error>(id);
        }
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Sender<protocol, tls_policy>::send(Message_buffer&& msg)
    {
        if (!enabled) return;

        try {
            socket->send(std::move(msg));
        }
        catch (std::system_error& ex) {
            stdout_log << Logging_level::debug << "Sender - failure to send: " << ex.what() << endl;
            
            dispatcher->template notify<Event_traits::Tx_error>(id);
        }
    }


    // =================================================================================================================
    //
    template <Protocol protocol, Navtech::TLS::Policy tls_policy = TLS::Policy::none>
    class Receiver : public Utility::Active {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unweildy) these parameters are looked up from the Connection_traits
        // class, using the appropriate combination of protocol and tls_policy
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol>;
        using Connection_traits = Navtech::Networking::Connection_traits<protocol, tls_policy>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Socket_Ty         = typename Connection_traits::Socket;
        using ID_Ty             = typename Connection_traits::ID;
        using Message_Ty        = typename Connection_traits::Message;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        Receiver(Socket_Ty& sckt, ID_Ty identity, Dispatcher_Ty& event_dispatcher);

    private:
        // External service associations
        //
        association_to<Socket_Ty>      socket;
        association_to<Dispatcher_Ty>  dispatcher;

        // Operating state
        //
        bool enabled { false };
        ID_Ty   id;

        // Active class overrides
        //
        void on_start() override;
        void on_stop()  override;

        // Buffers for incoming data
        //
        Message_Ty incoming_msg { };

        Active::Task_state run() override;
        void dispatch();
        void shutdown();
    };


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    Receiver<protocol, tls_policy>::Receiver(
        Receiver<protocol, tls_policy>::Socket_Ty& sckt, 
        ID_Ty identity,
        Receiver<protocol, tls_policy>::Dispatcher_Ty& event_dispatcher
    ) :
        Active      { "Receiver" },
        socket      { associate_with(sckt) },
        dispatcher  { associate_with(event_dispatcher) },
        id          { identity }
    {
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Receiver<protocol, tls_policy>::on_start()
    {
        enabled = true;
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Receiver<protocol, tls_policy>::on_stop()
    {
        enabled = false;
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    Active::Task_state Receiver<protocol, tls_policy>::run()
    {
        if (!enabled) {
            return Task_state::finished;
        }
        
        if (!socket->is_open()) {
            shutdown();
            return Task_state::finished;
        }

        try {

            Connection_traits::receive_payload(socket, incoming_msg);

            if (Protocol_traits::is_complete(incoming_msg)) {
                if (Protocol_traits::is_valid(incoming_msg)) {
                    dispatch();
                }
                else {
                    Connection_traits::discard_header(socket, incoming_msg);
                }
            }

            return Task_state::not_finished;
        }
        catch (std::system_error& e) {
            stdout_log << Logging_level::debug << "Receiver - failure to read byte stream: " << e.what() << endl;
            shutdown();
            return Task_state::finished;
        }
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Receiver<protocol, tls_policy>::dispatch()
    {
        Protocol_traits::add_client_id(incoming_msg, id);
        dispatcher->template notify<Event_traits::Received_message>(std::move(incoming_msg));
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Receiver<protocol, tls_policy>::shutdown()
    {
        dispatcher->template notify<Event_traits::Rx_error>(id);
    }


    // =================================================================================================================
    //
    template <Protocol protocol, Navtech::TLS::Policy tls_policy= TLS::Policy::none>
    class Connection {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unweildy) these parameters are looked up from the Connection_traits
        // class, using the appropriate combination of protocol and tls_policy
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol>;
        using Connection_traits = Navtech::Networking::Connection_traits<protocol, tls_policy>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Socket_Ty         = typename Connection_traits::Socket;
        using Message_Ty        = typename Connection_traits::Message;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        Connection(ID_Ty identifier, Socket_Ty&& sckt, Dispatcher_Ty& event_dispatcher);
        ~Connection();

        void open();
        void close();

        void bind_to(const Endpoint& local_endpt);
        void remote_endpoint(const Endpoint& remote_endpt);

        void send(const Message_Ty& msg);
        void send(Message_Ty&& msg);

    private:
        // External associations
        //
        association_to<Dispatcher_Ty> dispatcher;

        // Internal components
        //
        Socket_Ty socket { };
        Sender<protocol, tls_policy>   sender   { socket };
        Receiver<protocol, tls_policy> receiver { socket };

        // Event handling (from sender and receiver)
        //
        Utility::Event_handler<ID_Ty> error_handler { };
        void on_error(const ID_Ty& identity);

        // Internal state
        //
        ID_Ty   id      { };
        bool enabled { false };
    };



    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    Connection<protocol, tls_policy>::Connection(
        Connection<protocol, tls_policy>::ID_Ty            identity, 
        Connection<protocol, tls_policy>::Socket_Ty&&      sckt,
        Connection<protocol, tls_policy>::Dispatcher_Ty&   event_dispatcher
    ) :
        dispatcher  { associate_with(event_dispatcher) },
        socket      { std::move(sckt) },
        sender      { socket, identity, event_dispatcher },
        receiver    { socket, identity, event_dispatcher },
        id          { identity }
    {
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    Connection<protocol, tls_policy>::~Connection()
    {
        close();
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Connection<protocol, tls_policy>::open()
    {
        enabled = true;

        error_handler.when_notified_invoke(&Connection::on_error, this);
        dispatcher->template attach_to<Event_traits::Tx_error>(error_handler);
        dispatcher->template attach_to<Event_traits::Rx_error>(error_handler);

        sender.start();
        receiver.start();
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Connection<protocol, tls_policy>::close()
    {
        if (!enabled) return;
        enabled = false;

        dispatcher->template detach_from<Event_traits::Tx_error>(error_handler);
        dispatcher->template detach_from<Event_traits::Rx_error>(error_handler);

        socket.close();

        sender.stop();

        receiver.stop();
        receiver.join();
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Connection<protocol, tls_policy>::on_error(const ID_Ty& session_in_error)
    {
        if (session_in_error != this->id) return;

        // The receiver of the connection_error event 
        // should terminate this connection.
        //
        dispatcher->template notify<Event_traits::Connection_error>(id);
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Connection<protocol, tls_policy>::bind_to(const Endpoint& local_endpt)
    {
        socket.bind_to(local_endpt);
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Connection<protocol, tls_policy>::remote_endpoint(const Endpoint& remote_endpt)
    {
        socket.remote_endpoint(remote_endpt);
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Connection<protocol, tls_policy>::send(const Connection<protocol, tls_policy>::Message_Ty& msg)
    {
        if (!enabled) return;

        sender.send(Protocol_traits::to_buffer(msg));
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Connection<protocol, tls_policy>::send(Connection<protocol, tls_policy>::Message_Ty&& msg)
    {
        if (!enabled) return;

        sender.send(Protocol_traits::to_buffer(std::move(msg)));
    }


} // Navtech::Networking::Bytestream

#endif // BYTESTREAM_CONNECTION_H