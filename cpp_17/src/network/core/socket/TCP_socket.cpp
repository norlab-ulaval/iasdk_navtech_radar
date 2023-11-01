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

#ifdef __linux__
#include <arpa/inet.h>
#elif _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include <system_error>

#include "TCP_socket.h"

#include "net_conversion.h"
#include "Memory_types.h"

using std::size_t;
using std::vector;
using std::uint8_t;
using std::int32_t;
using std::system_error;
using std::error_code;
using std::system_category;
using namespace Navtech::Unit;

namespace Navtech::Networking {

    // -------------------------------------------------------------------------
    // Construction / destruction
    //
    TCP_socket::TCP_socket() : Socket { ::socket(AF_INET, SOCK_STREAM, 0) }
    {
        recv_buffer.reserve(64_kB);
    }


    TCP_socket::TCP_socket(Socket::Native_handle socket_handle, const Endpoint& endpt) :
        Socket { socket_handle, endpt }
    {
    }


    // -------------------------------------------------------------------------
    // Read/write interface
    //
    size_t TCP_socket::send(const vector<uint8_t>& buffer)
    {
        int32_t flags { };
    #ifdef __linux__
        flags |= MSG_NOSIGNAL;
	#endif

        auto result = ::send(
            native_handle(),
            buffer.data(),
            buffer.size(),
            flags
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket send" };

        return result;
    }


    size_t TCP_socket::send(vector<uint8_t>&& buffer)
    {
        // Take ownership for send
        //
        vector<uint8_t> send_buffer { std::move(buffer) };
        return send(send_buffer);
    }


    vector<uint8_t> TCP_socket::receive(size_t num_bytes)
    {
        return receive(num_bytes, Socket::Read_mode::consume);
    }



    vector<uint8_t> TCP_socket::receive(size_t num_bytes, Read_mode mode)
    {
        int32_t flags { };
        if (mode == Socket::Read_mode::peek) flags |= MSG_PEEK;

        recv_buffer.resize(num_bytes);

        uint8_t* insert_ptr      { recv_buffer.data() };
        size_t   bytes_remaining { num_bytes };
        size_t   bytes_read      { 0 };

        do {
            if (!is_open()) break;

            auto result = ::recv(
                native_handle(),
                insert_ptr,
                bytes_remaining,
                flags
            );

            if (result < 0) throw system_error { errno, system_category(), "TCP Socket receive" };

            bytes_read += result;
            insert_ptr += bytes_read;
            bytes_remaining = num_bytes - bytes_read;

        } while (bytes_remaining > 0);

        return recv_buffer;
    }


    // -------------------------------------------------------------------------
    // Connection interface
    //
    void TCP_socket::bind_to(const Endpoint& endpt)
    {
        // Don't allow binding to port 0; event though this is allowed
        // by Posix sockets
        //
        if (endpt.port == 0) throw system_error { EFAULT, system_category(), "TCP Socket bind" };

        auto addr = endpt.to_sockaddr();

        auto result = ::bind(
            native_handle(), 
            reinterpret_cast<struct sockaddr*>(&addr), 
            sizeof(addr)
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket bind" };

        local_endpoint(endpt);
        ready();
    }

    Endpoint TCP_socket::peer() const noexcept
    {
        struct sockaddr addr            { };
		socklen_t       size            { sizeof(addr) };
        Endpoint        client_endpoint { };

		auto result = ::getpeername(native_handle(), &addr, &size);
	
        if (result == 0) {
            client_endpoint = Endpoint { addr, local_endpoint().port };
        }

		return client_endpoint;
    }


    void TCP_socket::listen(uint8_t max_connections)
    {
        if (!is_open()) throw system_error { EBADFD, system_category(), "TCP Socket listen" };

        auto result = ::listen(
            native_handle(), 
            max_connections
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket listen" };
    }


    TCP_socket TCP_socket::accept()
    {
        auto addr    = local_endpoint().to_sockaddr();
        auto addr_sz = sizeof(addr);

        auto result = ::accept(
            native_handle(), 
            reinterpret_cast<struct sockaddr*>(&addr), 
            reinterpret_cast<socklen_t*>(&addr_sz)
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket accept" };

        return { result, local_endpoint() };
    }


    void TCP_socket::connect_to(const Endpoint& endpt)
    {
        auto addr = endpt.to_sockaddr();

        auto result = ::connect(
            native_handle(), 
            reinterpret_cast<struct sockaddr*>(&addr), 
            sizeof(addr)
        );

        if (result < 0) throw system_error { errno, system_category(), "TCP Socket connect" };

        local_endpoint(endpt);
        ready();
    }

} // namespace Navtech::Networking