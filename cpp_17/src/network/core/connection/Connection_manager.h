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

#ifndef CONNECTION_MANAGER_H
#define CONNECTION_MANAGER_H

#include <unordered_map>
#include <set>
#include <mutex>
#include <memory>

#include "Connection_manager_traits.h"
#include "Event_traits.h"
#include "pointer_types.h"
#include "Log.h"


using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using Navtech::Utility::pop_level;

namespace Navtech::Networking {

    // ----------------------------------------------------------------------------
    // The Connection_manager controls the lifetime of Connection objects, and provides
    // an interface to give access to the current set of Connections.
    //
    template <Protocol protocol, TLS::Policy tls_policy = TLS::Policy::none>
    class Connection_manager {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unweildy) these parameters are looked up from the Connection_traits
        // class, using the appropriate combination of protocol and tls_policy
        //
        using Event_traits      = Navtech::Networking::Event_traits<protocol>;
        using Connection_traits = Connection_manager_traits<protocol, tls_policy>;
        using Protocol_traits   = typename Connection_traits::Protocol_traits;
        using Connection_Ty     = typename Connection_traits::Connection;
        using Socket_Ty         = typename Connection_traits::Socket;
        using ID_Ty             = typename Connection_traits::ID;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;

        Connection_manager(Dispatcher_Ty& event_dispatcher);
        ~Connection_manager();

        void create_connection(Socket_Ty&& sckt);
        association_to<Connection_Ty> connection(ID_Ty id);
        association_to<Connection_Ty> connection();
        std::vector<association_to<Connection_Ty>> all_connections();
        bool remove(ID_Ty id);
        std::size_t num_connections() const;

    private:
        // External associations
        //
        association_to<Dispatcher_Ty> dispatcher;

        std::unordered_map<ID_Ty, shared_owner<Connection_Ty>> connections { };
        std::set<Networking::IP_address> well_known_clients { };
        
        ID_Ty next_id();
        bool accept_connection(const Socket_Ty& socket);
        void remove_all();
        
    };


    // ----------------------------------------------------------------------------

    template <Protocol protocol, TLS::Policy tls_policy>
    Connection_manager<protocol, tls_policy>::Connection_manager(
        Connection_manager<protocol, tls_policy>::Dispatcher_Ty& event_dispatcher
    ) :
        dispatcher { associate_with(event_dispatcher) }
    {
        // Establish the set of 'well known' clients
        // that are always allowed to connect and start
        // a connection
        //
        well_known_clients.emplace("127.0.0.1");       // Localhost
        well_known_clients.emplace("169.254.69.2");    // On-board Tracker
    }
    

    template <Protocol protocol, TLS::Policy tls_policy>
    Connection_manager<protocol, tls_policy>::~Connection_manager()
    {
        remove_all();
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Connection_manager<protocol, tls_policy>::create_connection(Connection_manager<protocol, tls_policy>::Socket_Ty&& sckt)
    {
        if (!accept_connection(sckt)) return;

        auto id = next_id();

        auto inserted = connections.emplace(id, allocate_shared<Connection_Ty>(id, std::move(sckt), *dispatcher)); 
        inserted.first->second->open();

        stdout_log << "Created new connection [" << id << "] "
                   << "Number of connections [" << connections.size() << "]"
                   << endl;

        // Notify anyone who might be interested in the new connection
        //
        dispatcher->template notify<Event_traits::Client_connected>(id);
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    bool Connection_manager<protocol, tls_policy>::accept_connection(const Connection_manager<protocol, tls_policy>::Socket_Ty& socket)
    {
        // Always allow 'well known' clients to connect
        //
        if (well_known_clients.count(socket.peer().ip_address) == 1) {
            stdout_log << Logging_level::error
                       << "Connection from well-known address " 
                       << "[" << socket.peer().to_string() << "]" 
                       << pop_level << endl;
            
            return true;
        }

        // Only allow a fixed number of clients
        //
        if (connections.size() >= Connection_traits::max_clients) {
            stdout_log << Logging_level::error
                       << "Connected rejected - maximum possible connections "
                       << "[" << Connection_traits::max_clients << "]"
                       << pop_level << endl;
            
            return false;
        }

        return true;
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    association_to<typename Connection_manager<protocol, tls_policy>::Connection_Ty> 
    Connection_manager<protocol, tls_policy>::connection(Connection_manager<protocol, tls_policy>::ID_Ty id)
    {
        if (auto it = connections.find(id); it != end(connections)) {
            return it->second.get();
        }
        else {
            return nullptr;
        }
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    association_to<typename Connection_manager<protocol, tls_policy>::Connection_Ty> 
    Connection_manager<protocol, tls_policy>::connection()
    {
        if (connections.size() != 1) return nullptr;
        return all_connections()[0];
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    std::vector<association_to<typename Connection_manager<protocol, tls_policy>::Connection_Ty>> 
    Connection_manager<protocol, tls_policy>::all_connections()
    {
        std::vector<association_to<Connection_manager<protocol, tls_policy>::Connection_Ty>> results { };

        for (auto& connection_pair : connections) {
            results.push_back(connection_pair.second.get());
        }

        return results;
    }
    

    template <Protocol protocol, TLS::Policy tls_policy>
    bool Connection_manager<protocol, tls_policy>::remove(Connection_manager<protocol, tls_policy>::ID_Ty id)
    {
        auto it = connections.find(id);
        if (it != end(connections)) {
            connections.erase(it);

            stdout_log << "Removing connection [" << id << "] "
                       << "Number of connections [" << connections.size() << "]"
                       << endl;

            // Notify anyone who might be interested in the removal
            // of this connection
            //
            dispatcher->template notify<Event_traits::Client_disconnected>(id);
            
            return true;
        }
        else {
            return false;
        }
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    void Connection_manager<protocol, tls_policy>::remove_all()
    {
        // Removing elements from the connections map will
        // invalid iterators; so build a vector of IDs to
        // remove, then remove each one individually.
        //
        std::vector<ID_Ty> to_remove { };

        for (const auto& connection_pair : connections) {
            to_remove.emplace_back(connection_pair.first);
        }

        for (auto id : to_remove) {
            remove(id);
        }
    }


    template <Protocol protocol, TLS::Policy tls_policy>
    typename Connection_manager<protocol, tls_policy>::ID_Ty Connection_manager<protocol, tls_policy>::next_id()
    {
        // Connection ID_Ty 0 (zero) is configured as the 'broadcast'
        // ID_Ty - that is, send to all available connections
        //
        static Connection_manager<protocol, tls_policy>::ID_Ty id { 1 };
        return id++;
    }


    template <Protocol protocol, TLS::Policy tls_policy> 
    std::size_t Connection_manager<protocol, tls_policy>::num_connections() const
    {
        return connections.size();
    }

} // namespace Navtech::Networking

#endif // CONNECTION_MANAGER_H