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

#include "Colossus_test_runner.h"
#include "Log.h"

namespace Navtech::Networking::Colossus_protocol {

    Test_runner::Test_runner(const Endpoint& radar, std::initializer_list<Test> tests_init) :
        radar_address   { radar },
        tests           { tests_init }
    {
    }


    void Test_runner::add_test(const Test& test)
    {
        tests.push_back(test);
    }

    
    void Test_runner::run()
    {
        for (auto& test : tests) run_test(test);
    }


    void Test_runner::run(const std::string& test_str)
    {
        if (test_str == "all") run();

        auto itr = std::find_if(
            tests.begin(),
            tests.end(),
            [&test_str](const Test& t) { return t == test_str; }
        );

        if (itr != tests.end()) run_test(*itr);
    }


    void Test_runner::run_test(Test& test)
    {
        stdout_log << "------------------------------------------------" << endl;
        stdout_log << "RUNNING TEST: " << test.description() << endl;

        radar_client = allocate_owned<Radar_client>(radar_address);
        test.set_radar_client(*radar_client);
        test.run();
        radar_client->stop();
        radar_client.reset();
    }


    void Test_runner::display_results() const
    {
        using Navtech::Utility::stdout_log;
        using Navtech::Utility::endl;
        using Navtech::Utility::Logging_level;

        int passed  { };
        int failed  { };
        int unknown { };
        int not_run { };

        stdout_log.time_format("");
        stdout_log.min_level(Logging_level::info);

        stdout_log << "------------------------------------------------" << endl;
        stdout_log << "Test results" << endl;
        stdout_log << "============" << endl;

        for (const auto& test : tests) {
            stdout_log << test.result_str() << endl;

            if (test.result() == Test::pass)            ++passed;
            else if (test.result() == Test::fail)       ++failed;
            else if (test.result() == Test::unknown)    ++unknown;
            else                                        ++not_run;
        }

        stdout_log << Logging_level::info << endl;
        stdout_log << "Total tests: " << tests.size() << endl;
        stdout_log << "Passed:      " << passed  << endl;
        stdout_log << "Failed:      " << failed  << endl;
        stdout_log << "Unknown:     " << unknown << endl;
        stdout_log << "Not run:     " << not_run << endl;
        stdout_log << "------------------------------------------------" << endl;
    }

} // namespace Navtech::Networking::Colossus_protocol