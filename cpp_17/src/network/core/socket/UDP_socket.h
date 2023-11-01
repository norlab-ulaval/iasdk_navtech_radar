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

#ifndef UDP_SOCKET_H
#define UDP_SOCKET_H

#ifdef __linux__
#include <arpa/inet.h>
#include <unistd.h>
#elif _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include <utility>

#include "Socket.h"
#include "Endpoint.h"
#include "Message_buffer.h"

#include "pointer_types.h"

namespace Navtech::Networking {

    // -----------------------------------------------------------------------------
    // A UDP_socket is a datagram-based socket
    //
    class UDP_socket : public Socket {
    public:
        UDP_socket();
        UDP_socket(const Endpoint& local_endpt);
        UDP_socket(const Endpoint& local_endpt, const Endpoint& remote_endpt);
        UDP_socket(const Endpoint& local_endpt, const Endpoint& remote_endpt, std::uint8_t TTL);
        // Copy/move policy
        //
        UDP_socket(const UDP_socket&)               = delete;
        UDP_socket& operator=(const UDP_socket&)    = delete;
        UDP_socket(UDP_socket&&);           
        UDP_socket& operator=(UDP_socket&&);

        // NOTE: All functions will throw a std::system_error
        // exception on failure; except where noted.
        
        // Read/write interface
        //
        std::size_t send(const Message_buffer& buffer);
        std::size_t send(Message_buffer&& buffer);

        std::pair<IP_address, Message_buffer> receive(std::size_t num_bytes);
        std::pair<IP_address, Message_buffer> receive(std::size_t num_bytes, Read_mode mode);

        // Connection interface.
        //
        void bind_to(const Endpoint& endpt);
        void remote_endpoint(const Endpoint& endpt);

        void join_group(const Endpoint& group);

        Endpoint peer() const noexcept;
       
    protected:
        UDP_socket(Socket::Native_handle socket_handle, const Endpoint& endpt);

    private:
        Message_buffer recv_buffer { };

        Endpoint     remote_endpt { };
        std::uint8_t ttl          { 1 };

        // Helpers
        //
        bool is_multicast_address(const Endpoint& addr) const;
        void enable_multicast();
    };


} // namespace Navtech::Networking

#endif // UDP_SOCKET_H