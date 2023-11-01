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

#ifndef ENCRYPTED_CONNECTION_H
#define ENCRYPTED_CONNECTION_H

#include "Connection_traits.h"
#include "Active.h"
#include "pointer_types.h"
#include "Log.h"
#include "Time_utils.h"

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;
using namespace Navtech::Time;

namespace Navtech::Networking::Secure {

    // =================================================================================================================
    //
    template <Protocol protocol, Navtech::TLS::Policy tls_policy = TLS::Policy::none>
    class Secure_connection {
    public:
        Secure_connection(ID identifier, Socket&& sckt);
        ~Secure_connection();

        void open();
        void close();

        void send(const Message& msg);
        void send(Message&& msg);

    private:
    };


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    Secure_connection<protocol, tls_policy>::Secure_connection(ID, Socket&&)
    {
        // TODO - 
    }

    
    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    Secure_connection<protocol, tls_policy>::~Secure_connection()
    {
        // TODO - 
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Secure_connection<protocol, tls_policy>::open()
    {
        // TODO - 
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Secure_connection<protocol, tls_policy>::close()
    {
        // TODO - 
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Secure_connection<protocol, tls_policy>::send(const Message&)
    {
        // TODO - 
    }


    template <Protocol protocol, Navtech::TLS::Policy tls_policy>
    void Secure_connection<protocol, tls_policy>::send(Message&&)
    {
        // TODO - 
    }

} // namespace Navtech::Networking::Secure

#endif // ENCRYPTED_CONNECTION_H