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

#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H

#include "Socket.h"
#include "Endpoint.h"

namespace Navtech::Networking {

    // -----------------------------------------------------------------------------
    // A TCP_socket is a connection-based socket
    //
    class TCP_socket : public Socket {
    public:
        TCP_socket();

        // NOTE: All functions will throw a std::system_error
        // exception on failure; except where noted.
        
        // Read/write interface
        //
        std::size_t send(const std::vector<std::uint8_t>& buffer);
        std::size_t send(std::vector<std::uint8_t>&& buffer);

        std::vector<std::uint8_t> receive(std::size_t num_bytes);
        std::vector<std::uint8_t> receive(std::size_t num_bytes, Read_mode mode);

        // Connection interface
        //
        void bind_to(const Endpoint& endpt);
        void listen(std::uint8_t max_connections);
        TCP_socket accept();
        
        void connect_to(const Endpoint& endpt);

        Endpoint peer() const noexcept;
        
    protected:
        TCP_socket(Socket::Native_handle socket_handle, const Endpoint& endpt);

    private:
        std::vector<std::uint8_t> recv_buffer { };
    };

} // namespace Navtech::Networking

#endif // TCP_SOCKET_H