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

#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <filesystem>


namespace Navtech::TLS {

    // ----------------------------------------------------------------------------
    // The Configuration structure holds the settings for the networking
    // components.
    // The Configuration can be saved/read-from a file, in protocol buffer
    // format.
    // The defaults defined will be over-written if a settings file exists.
    //
    struct Configuration {
        std::filesystem::path filename              { "/systemconfig/tls.settings" };
        std::filesystem::path certificate_filename  { "/systemconfig/certs/device.crt" };
        std::filesystem::path key_filename          { "/systemconfig/certs/device.key" };
        std::string           passphrase            { "" };

        void create();
        void store();
        bool load();
    };

} // namespace Navtech::TLS

#endif // CONFIG_H