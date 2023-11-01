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
#include <sstream>

#include "Option_parser.h"
#include "Log.h"

namespace Navtech::Utility {
    // -----------------------------------------------------------------------------------------------------------------
    //
    Option::Option(
        std::string_view    name,
        std::string_view    short_name,
        std::string_view    description_text,
        Optionality         reqd_or_opt,
        Argument            has_argument,
        std::string_view    argument_default
    ) :
        long_form       { remove_prefix(name) },
        short_form      { remove_prefix(short_name) },
        description     { description_text },
        is_optional     { reqd_or_opt  == Optionality::optional },
        has_arg         { has_argument == Argument::has_argument },
        arg_default     { argument_default },
        arg_value       { arg_default }
    {
    }


    std::string Option::value() const
    {
        return arg_value;
    }


    std::string Option::help() const
    {
        std::stringstream ss { };

        ss << (is_optional ? "[Optional] " : "[Required] ");
        ss << "--" << long_form;
        ss << ", ";
        ss << "-" << short_form << "";
        ss << " : " << description << " ";

        if (has_arg) {
            if (!arg_default.empty()) {
                ss << "[Default: " << arg_default << "]";
            }
        }

        return ss.str();
    }


    std::string Option::usage() const
    {
        std::stringstream ss { };

        if (is_optional) ss << "[";

        ss << "--" << long_form << " ";
        if (has_arg) ss << "<val>";

        ss << " | ";

        ss << "-" << short_form << " ";
        if (has_arg) ss << "<val>";

        if (is_optional) ss << "]";

        return ss.str();
    }



    void Option::parse(std::vector<std::string>& option_strings)
    {
        // Find this Option and any required argument in the
        // supplied strings.
        // Throw an exception if the option is required, but
        // not in the list; or if the option requires an argument
        // but none is present.

        auto opt_iter = std::find_if(
            option_strings.begin(),
            option_strings.end(),
            [this](const std::string& token)
            {
                auto naked_token = remove_prefix(token);
                return (naked_token == long_form) || (naked_token == short_form);
            }
        );

        bool opt_found { opt_iter != option_strings.end() };

        if (opt_found) {
            // Get the argument, if there should be one.
            // The argument should be the next string in the vector.
            //
            if (has_arg) {
                auto arg_iter = opt_iter + 1;

                if (arg_iter == option_strings.end()) {
                    // Oops, run out of option strings!
                    //
                    throw std::invalid_argument { "Missing the required argument for --" + long_form };
                }

                if ((*arg_iter)[0] == '-') {
                    // Ooops, found the next option, not the argument!
                    //
                    throw std::invalid_argument { "Missing the required argument for --" + long_form };
                }

                arg_value = *arg_iter;
            }
        }
        else {
            if (!is_optional) throw std::invalid_argument { "Required option --" + long_form + " is missing" };

        }
    }


    bool Option::operator==(std::string_view option_name) const
    {
        auto raw_name = remove_prefix(option_name);
        return ((raw_name == long_form) || (raw_name == short_form));
    }


    std::string Option::remove_prefix(std::string_view prefixed_name) const
    {
        std::string name { prefixed_name };

        // Remove leading hyphens from the name
        // to leave a 'raw' identifier
        //
        name.erase(
            std::remove_if(
                name.begin(), 
                name.end(),
                [](const char chr) { return (chr == '-'); }
            ),
            name.end()
        );

        return name;
    }

    // -----------------------------------------------------------------------------------------------------------------
    //
    Option_parser::Option_parser(std::string_view exe_name) :
        name { exe_name }
    {
    }


    Option_parser::Option_parser(std::string_view exe_name, std::initializer_list<Option> option_list) :
        name    { exe_name },
        options { option_list }
    {
    }


    Option_parser& Option_parser::add_option(const Option& option)
    {
        options.push_back(option);
        return *this;
    }


    Option_parser& Option_parser::add_option(Option&& option)
    {
        options.push_back(std::move(option));
        return *this;
    }


    void Option_parser::parse(int argc, char* argv[])
    {
        std::vector<std::string> tokens { };

        for (int i { 1 }; i < argc; ++i) {
            tokens.emplace_back(argv[i]);
        }

        try {
            for (auto& opt : options) {
                opt.parse(tokens);
            }
        }
        catch (std::exception& ex) {
            std::cout << std::endl;
            std::cout << "ERROR - " << ex.what() << std::endl;
            std::cout << "Usage:" << std::endl;
            
            std::cout << "\t" << name << " ";
            for (const auto& option : options) {
                std::cout << option.usage() << " ";
            }
            std::cout << std::endl;
            std::cout << std::endl;

            std::cout << "Options:" << std::endl;
            for (const auto& option : options) {
                std::cout << "\t" <<  option.help() << std::endl;
            }

            std::cout << std::endl;

            exit(-1);
        }
    }


    const Option& Option_parser::operator[](std::string_view option)
    {
        auto itr = std::find(options.begin(), options.end(), option);

        if (itr == options.end()) {
            throw std::invalid_argument { "Unknown option: " + std::string { option } };
        }

        return *itr;
    }
    

} // namespace Navtech::Utility