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

#ifndef SOCKET_H
#define SOCKET_H

#ifdef __linux__
#include <arpa/inet.h>
#include <unistd.h>
#elif _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include <cstdint>
#include <cstddef>
#include <vector>
#include <system_error>
#include <mutex>

#include "Endpoint.h"
#include "Time_utils.h"

namespace Navtech::Networking {

    // -----------------------------------------------------------------------------
    // Socket is the base class for all socket-type connections.
    // It defines the common elements of all sockets.
    // Note: as Sockets will not be used in substitution heirarchies
    // the Socket class does not define any virtual functions
    //
    class Socket {
    public:
        using Native_handle = int;
    
        enum Read_mode { consume, peek };

        Socket() = default;
        ~Socket();

        // Copy / move policy
        //
        Socket(const Socket&)               = delete;
        Socket& operator=(const Socket&)    = delete;
        Socket(Socket&&);
        Socket& operator=(Socket&&);

        bool is_open() const noexcept;
        void close() noexcept;
        bool valid() noexcept;
        operator bool() noexcept;
        Native_handle native_handle() const noexcept;
        
        const Endpoint& local_endpoint() const;

        void tx_timeout(const Time::Duration& timeout);
        Time::Duration tx_timeout() const;

        void rx_timeout(const Time::Duration& timeout);
        Time::Duration rx_timeout() const;

        void tx_buffer(std::size_t sz);
        void rx_buffer(std::size_t sz);

        void keep_alive(bool enable);
        bool keep_alive() const;

        void reuse_address();
        void linger();


    protected:
        // Clients cannot create new Sockets from a handle
        //
        Socket(Native_handle socket_handle);
        Socket(Native_handle socket_handle, const Endpoint& endpt);

        void local_endpoint(const Endpoint& endpt);

        void ready();
        bool is_valid() const;

    private:
        Native_handle handle  { -1 };  // Invalid.
        Endpoint      address { };

        mutable std::mutex mtx { };
        bool open { false };

        void set_options();
    };

} // namespace Navtech::Networking

#endif // SOCKET_H