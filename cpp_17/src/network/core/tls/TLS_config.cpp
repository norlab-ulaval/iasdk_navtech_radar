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

#include <filesystem>
#include <fstream>

#include "TLS_config.h"

// -----------------------------------------------------------
// NOTE - Protocol buffer store/load has been removed from
// this code, for simplicity.
// All configuration settings are the defaults
//
// -----------------------------------------------------------

namespace Navtech::TLS {

    void Configuration::create()
    {
        // Write the default settings
        // to a protocol buffer.
        //
        store();
    }


    void Configuration::store()
    {
        

        // Write current configuration to
        // a protocol buffer and stream to a file
        //
        // ::TLS::TLS_settings settings_protobuf { };
        // settings_protobuf.set_server_certificate(certificate_filename);
        // settings_protobuf.set_server_key(key_filename);
        // settings_protobuf.set_server_key_passphrase(passphrase);

        // std::fstream config_file(filename, std::ios::out | std::ios::trunc | std::ios::binary);
        // settings_protobuf.SerializeToOstream(&config_file);
    }


    bool Configuration::load()
    {
        // if (!std::filesystem::exists(filename)) return false;

        //  std::fstream config_file(filename, std::ios::in | std::ios::binary);
        // ::TLS::TLS_settings settings_protobuf { };

        // if (!settings_protobuf.ParseFromIstream(&config_file)) return false;

        // certificate_filename = settings_protobuf.server_certificate();
        // key_filename         = settings_protobuf.server_key();
        // passphrase           = settings_protobuf.server_key_passphrase();
        
        return true;
    }



} // namespace Navtech::TLS::Botan