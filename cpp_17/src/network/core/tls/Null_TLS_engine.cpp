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

#include <algorithm>
#include <iterator>

#include "Null_TLS_engine.h"
#include "Null_TLS_services.h"


namespace Navtech {

    namespace TLS::Null {

        // ------------------------------------------------------------------------------------------
        // TLS_engine
        //
        TLS_engine::TLS_engine(TLS_services& tls_services [[maybe_unused]]) 
        {
            // Wire up the message input/output ports to provide 
            // a simple pass-through of data.
            //
            encrypt_in.forward_to(encrypt_out);
            decrypt_in.forward_to(decrypt_out);
        }


        void TLS_engine::open()
        {
            status << state_change(Networking::Component::tls, Networking::State::open);
        }
        
        
        void TLS_engine::close()
        {
            status << state_change(Networking::Component::tls, Networking::State::closed);
        }


    } // namespace TLS::Null

} // namespace Navtech