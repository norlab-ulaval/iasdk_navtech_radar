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

#ifndef OPTION_PARSER_H
#define OPTION_PARSER_H

#include <string>
#include <vector>
#include <optional>
#include <string_view>
#include <functional>
#include <initializer_list>
#include <type_traits>
#include <sstream>

namespace Navtech::Utility {

    enum Argument    { no_argument, has_argument };
    enum Optionality { optional, required };

    class Option {
    public:
        Option(
            std::string_view    name,
            std::string_view    short_name,
            std::string_view    help_text,
            Optionality         reqd_or_opt,
            Argument            has_argument,
            std::string_view    argument_default = ""
        );

        std::string value() const;

        std::string help() const;
        std::string usage() const;
        void parse(std::vector<std::string>& option_strings);

        bool operator==(std::string_view option_name) const;

        template <typename T>
        T translate_to(std::function<T(const std::string&)> translator_fn) const
        {
            static_assert(std::is_default_constructible<T>::value);

            if (arg_value.empty()) return T { };
            if (translator_fn) return translator_fn(arg_value);
            return T { };
        }

        template <typename T>
        T translate_to() const
        {
            static_assert(std::is_default_constructible<T>::value);

            if (arg_value.empty()) return T { };
            return T { arg_value };
        }

        template <typename T = int>
        T to_int() const
        {
            if (arg_value.empty()) return { };
            return static_cast<T>(std::stoi(arg_value));
        }

        int to_int() const
        {
            if (arg_value.empty()) return { };
            return std::stoi(arg_value);
        }

        bool to_bool() const
        {
            if (arg_value.empty()) return { };
            bool b { };
            std::istringstream(arg_value) >> std::boolalpha >> b;
            return b;
        }

    private:
        std::string long_form    { };
        std::string short_form   { };
        std::string description  { };
        bool        is_optional  { true };
        bool        has_arg      { false };
        std::string arg_default  { };
        std::string arg_value    { };

        std::string remove_prefix(std::string_view prefixed_name) const;
    };


    class Option_parser {
    public:
        Option_parser(std::string_view exe_name);
        Option_parser(std::string_view exe_name, std::initializer_list<Option> option_list);

        Option_parser& add_option(const Option& option);
        Option_parser& add_option(Option&& option);

        void parse(int argc, char* argv[]);

        const Option& operator[](std::string_view option);


    private:
        std::string         name    { };
        std::vector<Option> options { };
    };

} // namespace Navtech::Utility

#endif // OPTION_PARSER_H