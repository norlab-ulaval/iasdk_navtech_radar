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
#include <unistd.h>
#elif _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include <system_error>

#include "Socket.h"
#include "Memory_types.h"

using std::system_error;
using std::system_category;

namespace Navtech::Networking {

    // -------------------------------------------------------------------------
    // Construction / destruction
    //
    Socket::Socket(Socket::Native_handle fd) :
        handle { fd }
    {
        set_options();
    }


    Socket::Socket(Socket::Native_handle fd, const Endpoint& endpt) :
        handle      { fd },
        address    { endpt }
    {
        // This constructor must only be called as a result of
        // accepting the Socket on the server side.  This new
        // socket must, by definition, be ready to communicate.
        //
        set_options();
        ready();
    }


    Socket::~Socket()
    {
        close();
    }


    // -------------------------------------------------------------------------
    // Copy / move policy
    //
    Socket::Socket(Socket&& src) :
        handle    { std::move(src.handle) },
        address   { std::move(src.address) },
        open      { std::move(src.open) }
    {
        // NOTE - each Socket has its own unique mutex
        // which is never copied or moved.
    }
    
    
    Socket& Socket::operator=(Socket&& rhs)
    {
        handle  = std::exchange(rhs.handle, -1);
        address = std::exchange(rhs.address, Endpoint::null());
        open    = std::exchange(rhs.open, false);

        return *this;
    }


    // -------------------------------------------------------------------------
    // Public
    //
    bool Socket::is_open() const noexcept
    {
        std::lock_guard lock { mtx };
        return open;
    }


    bool Socket::valid() noexcept
    {
        return (handle != -1);
    }


    Socket::operator bool() noexcept
    {
        return valid();
    }


    void Socket::close() noexcept
    {
        std::lock_guard lock { mtx };
        
        if (!open)          return;
        if (handle == -1)   return;
        open = false;

        ::shutdown(handle, SHUT_RDWR);
        ::close(handle);

        handle = -1;
    }


    Socket::Native_handle Socket::native_handle() const noexcept
    {
        return handle;
    }


    const Endpoint& Socket::local_endpoint() const
    {
        return address;
    }


    void Socket::tx_timeout(const Time::Duration& timeout)
    {
        auto ntp_time = Time::Monotonic::Observation(timeout).to_ntp();
        
        ::setsockopt(
            handle, 
            SOL_SOCKET, 
            SO_SNDTIMEO, 
            reinterpret_cast<char*>(&ntp_time), 
            sizeof(ntp_time)
        );
    }


    Time::Duration Socket::tx_timeout() const
    {
        std::timespec ntp_time { };
        socklen_t     sz       { };

        ::getsockopt(
            handle,
            SOL_SOCKET,
            SO_SNDTIMEO,
            reinterpret_cast<void*>(&ntp_time),
            &sz
        );

        return Time::Duration { ntp_time };
    }


    void Socket::rx_timeout(const Time::Duration& timeout)
    {
        auto ntp_time = Time::Monotonic::Observation(timeout).to_ntp();
        
        ::setsockopt(
            handle, 
            SOL_SOCKET, 
            SO_RCVTIMEO, 
            reinterpret_cast<char*>(&ntp_time), 
            sizeof(ntp_time)
        );
    }


    Time::Duration Socket::rx_timeout() const
    {
        std::timespec ntp_time { };
        socklen_t     sz       { sizeof(ntp_time) };

        ::getsockopt(
            handle,
            SOL_SOCKET,
            SO_RCVTIMEO,
            reinterpret_cast<void*>(&ntp_time),
            &sz
        );

        return Time::Duration { ntp_time };
    }


    void Socket::tx_buffer(std::size_t sz)
    {
        ::setsockopt(
            handle,
            SOL_SOCKET,
            SO_SNDBUF,
            reinterpret_cast<void*>(&sz),
            sizeof(sz)
        );
    }


    void Socket::rx_buffer(std::size_t sz)
    {
        ::setsockopt(
            handle,
            SOL_SOCKET,
            SO_RCVBUF,
            reinterpret_cast<void*>(&sz),
            sizeof(sz)
        );
    }


    void Socket::keep_alive(bool enable)
    {
        int on { static_cast<int>(enable) };

       ::setsockopt(
            handle,
            SOL_SOCKET, 
            SO_KEEPALIVE, 
            reinterpret_cast<char*>(&on), 
            sizeof(on)
        );
    }
    
    
    bool Socket::keep_alive() const
    {
        int is_enabled { };
        socklen_t sz   { };

        ::getsockopt(
            handle,
            SOL_SOCKET,
            SO_KEEPALIVE,
            reinterpret_cast<void*>(&is_enabled),
            &sz
        );

        return is_enabled;
    }


    void Socket::reuse_address()
    {
        int on { 1 };
        auto result = ::setsockopt(
            handle,
            SOL_SOCKET, 
            SO_REUSEADDR, 
            reinterpret_cast<const char*>(&on), 
            sizeof(on)
        );

        if (result < 0) throw system_error { errno, system_category() };
    }


    void Socket::linger()
    {
        struct linger linger_val { };
		linger_val.l_onoff  = 0;
		linger_val.l_linger = 2;

		auto result = ::setsockopt(
            handle, 
            SOL_SOCKET, 
            SO_LINGER, 
            reinterpret_cast<const char*>(&linger_val), 
            sizeof(linger_val)
        );
	
        if (result < 0) throw system_error { errno, system_category() };
    }


    // -------------------------------------------------------------------------
    // Protected
    //    
    void Socket::local_endpoint(const Endpoint& endpt)
    {
        address = endpt;
    }


    void Socket::ready()
    {
        std::lock_guard lock { mtx };
        open = true;
    }


    bool Socket::is_valid() const
    {
        return (handle != -1);
    }

    // -------------------------------------------------------------------------
    // Private
    //
    void Socket::set_options()
    {
        if (!is_valid()) return;

        reuse_address();
        linger();
        
    }

} // namespace Navtech::Networking