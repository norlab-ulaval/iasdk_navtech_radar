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

#ifndef NULL_TLS_SERVICES_H
#define NULL_TLS_SERVICES_H

namespace Navtech::TLS::Null {
        
    // ---------------------------------------------------------------------------
    // TLS_services represents the shared/static TLS facilities required to
    // support individual TLS connections (as represented by the TLS_engine class)
    // TLS_services provides an abstract interface for facilities such as
    // - Certificate management
    // - Credentials management
    // - Reconnection state
    // - etc.  
    // Using a single object limits the need to pass multiple objects around; 
    // particularly when these components are often used in conjunction.
    //
    // A Null::TLS_services object provides no service; it is simply a placeholder
    // to support the TLS policy of the system.
    //
    class TLS_services {
    public:
    };

} // namespace Navtech::TLS::Null


#endif // NULL_TLS_SERVICES_H