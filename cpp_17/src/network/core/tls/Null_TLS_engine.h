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

#ifndef NULL_TLS_ENGINE_H
#define NULL_TLS_ENGINE_H


#include "Port.h"



namespace Navtech::TLS::Null {

    class Status;
    class Configuration;
    class TLS_services;
    
    // ----------------------------------------------------------------------------
    // The TLS_engine class encapsulates the TLS server components, and provides
    // an interface for sending/receiving data.
    // A Null::TLS_engine provides no encryption/decryption facilities.  It acts
    // as a placeholder to support the TLS policy of the system.
    //
    class TLS_engine {
    public:
        TLS_engine(TLS_services& tls_services);

        void open();
        void close();

        // Ports
        //
        // Networking::Buffer_port encrypt_in  { };
        // Networking::Buffer_port encrypt_out { };
        // Networking::Buffer_port decrypt_in  { };
        // Networking::Buffer_port decrypt_out { };
        // Networking::Status_port status      { };
        // Networking::Error_port  err         { };
    };

} // namespace Navtech:: TLS::Null


#endif // NULL_TLS_ENGINE_H