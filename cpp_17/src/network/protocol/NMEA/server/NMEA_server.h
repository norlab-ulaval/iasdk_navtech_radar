#ifndef NMEA_SERVER_H
#define NMEA_SERVER_H

#include "Datagram_server.h"
#include "NMEA_events.h"
#include "NMEA_protocol.h"

#include "Active.h"
#include "Endpoint.h"

namespace Navtech::Networking::NMEA_protocol {

    class Server : public Utility::Active {
    public:
        Server(const Endpoint& send_to);

        void send(const Message& msg);
        void send(Message&& msg);

    private:
        // Active class overrides
        //
        void on_start() override;
        void on_stop()  override;

        // Underlying server configuration
        //
        using Datagram_Server = Datagram_server<Protocol::nmea>;
        
        // NOTE: Do not change this declaration order!
        //
        Endpoint        local_endpt;
        Endpoint        remote_endpt;
        Datagram_Server datagram_server;
       
        void on_send(NMEA_protocol::Message& msg);

        // Helpers
        //
    };

} // namespace Navtech::Networking::NMEA_protocol


#endif // NMEA_SERVER_H