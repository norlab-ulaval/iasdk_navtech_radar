#include "NMEA_server.h"
#include "Message_buffer.h"


namespace Navtech::Networking::NMEA_protocol {

    Server::Server(const Endpoint& send_to) :
        Active          { "NMEA server" },
        local_endpt     { IP_address::any(), send_to.port },
        remote_endpt    { send_to },
        datagram_server { local_endpt, remote_endpt, 1, event_dispatcher }
    {
    }

    void Server::on_start()
    {
        datagram_server.start();
    }


    void Server::on_stop()
    {
        datagram_server.stop();
        datagram_server.join();
    }


    void Server::send(const NMEA_protocol::Message& msg)
    {
        async_call(&Server::on_send, this, msg);
    }


    void Server::send(NMEA_protocol::Message&& msg)
    {
        async_call(&Server::on_send, this, std::move(msg));
    }


    void Server::on_send(NMEA_protocol::Message& msg)
    {
        datagram_server.send(msg);
    }

} // namespace Navtech::Networking::NMEA_protocol