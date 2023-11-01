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

#ifndef COLOSSUS_TEST_RUNNER_H
#define COLOSSUS_TEST_RUNNER_H

#include <string>
#include <vector>
#include <initializer_list>

#include "Colossus_message_types.h"
#include "Colossus_test.h"
#include "Endpoint.h"

namespace Navtech::Networking::Colossus_protocol {

    
    class Test_runner {
    public:
        Test_runner() = default;
        Test_runner(const Endpoint& radar, std::initializer_list<Test> tests_init);

        void add_test(const Test& test);

        void run();
        void run(const std::string& test);
        void display_results() const;

    private:
        Endpoint               radar_address { };
        owner_of<Radar_client> radar_client { };
        std::vector<Test>      tests { };
        std::size_t            num_tests { };

        void run_test(Test& test);
    };


} // namespace Navtech::Networking::Colossus_protocol


#endif // COLOSSUS_TEST_RUNNER_H