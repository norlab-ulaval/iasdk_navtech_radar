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

#include <cstring>
#include <string>

#include "NMEA_message.h"

namespace Navtech::Networking::NMEA_protocol {

    
    // -----------------------------------------------------------------------------------------------
    // Construction
    //
    Message::Message(const Message_buffer& message_vector)
    {
        replace(message_vector);
    }


    Message::Message(Message_buffer&& message_vector)
    {
        replace(std::move(message_vector));
    }


    Message::Message(Message::Const_iterator message_start, std::size_t message_sz)
    {
        replace(message_start, message_sz);
    }


    Message::Message(std::string_view msg_str)
    {
        replace(reinterpret_cast<Const_iterator>(&msg_str[0]), msg_str.size());
    }

    // -----------------------------------------------------------------------------------------------
    // Copy/move policy
    // Copy/move functions must rebind the parser back to the (new) buffer;
    // otherwise they have an association to a buffer that's no longer valid
    //
    Message::Message(const Message& other) : 
        buffer      { other.buffer },
        identity    { other.identity },
        parser      { other.parser }
    {
        parser->rebind(buffer);
    }


    Message::Message(Message&& other) :
        buffer      { std::move(other.buffer) },
        identity    { std::move(other.identity) },
        parser      { std::move(other.parser) }
    {
        parser->rebind(buffer);
    }


    Message& Message::operator=(const Message& rhs)
    {
        buffer      = rhs.buffer;
        identity    = rhs.identity;
        parser      = rhs.parser;

        parser->rebind(buffer);

        return *this;
    }


    Message& Message::operator=(Message&& rhs)
    {
        buffer   = std::exchange(rhs.buffer, Message_buffer { });
        identity = std::exchange(rhs.identity, ID { });
        parser   = std::exchange(rhs.parser, nullptr);
        parser   = std::exchange(rhs.parser, nullptr);

        parser->rebind(buffer);

        return *this;
    }


    // -----------------------------------------------------------------------------------------------
    // 
    //
    void Message::client_id(Message::ID id)
    {
        identity = id;
    }
    
    
    Message::ID Message::client_id() const
    {
        return identity;
    }


    Type Message::type() const
    {
        if (!parser)                return Type::unknown;
        if (!parser->is_valid())    return Type::unknown;

        auto type_str = parser->type_as_string();

        if (type_str == "GPRMC")    return Type::gprmc;
        if (type_str == "GPHDT")    return Type::gphdt;
        if (type_str == "GPGGA")    return Type::gpgga;
        if (type_str == "PASHR")    return Type::pashr;
        if (type_str == "BESTPOS")  return Type::bestpos;

        return Type::unknown;
    }


    void Message::type(Type t)
    {
        append(std::string { NMEA_protocol::to_string(t) });
    }


    std::string Message::type_as_string() const
    {
        if (!parser)                return { };
        if (!parser->is_valid())    return { };

        return parser->type_as_string();
    }


    std::size_t Message::size() const
    {
        return buffer.size();
    }
    
    
    std::size_t Message::payload_size() const
    {
        if (!parser) return { };

        return (parser->payload_end() - parser->payload_begin());
    }


    // -----------------------------------------------------------------------------------------------
    // Iterators
    //
    Message::Iterator Message::begin()
    {
        if (!parser) return { };

        return parser->begin();
    }


    Message::Const_iterator Message::begin() const
    {
        if (!parser) return { };

        return parser->begin();
    }


    Message::Iterator Message::end()
    {
        if (!parser) return { };

        return parser->end();
    }


    Message::Const_iterator Message::end() const
    {
       if (!parser) return { };

        return parser->end();
    }


    Buffer_view Message::data_view() const
    {
        return Buffer_view { buffer };
    }


    Message::Iterator Message::payload_begin()
    {
        if (!parser) return { };

        return parser->payload_begin();
    }


    Message::Const_iterator Message::payload_begin() const
    {
        if (!parser) return { };

        return parser->payload_begin();
    }


    Message::Iterator Message::payload_end()
    {
       if (!parser) return { };

        return parser->end();
    }


    Message::Const_iterator Message::payload_end() const
    {
        if (!parser) return { };

        return parser->payload_end();
    }


    // -----------------------------------------------------------------------------------------------
    // Data insertion
    //
    Message& Message::append(const Message_buffer& data)
    {
        // - Remove the old payload sentinal and checksum
        // - Append the new data
        // - Recompute the checksum
        // - Add the payload sentinal and checksum
        // - Add the message delimiter (to keep things aligned)

        buffer.erase(buffer.begin() + payload_size() + 1, buffer.end());
        buffer.insert(buffer.end(), data.begin(), data.end());

        if (!parser) parser = Parser::get_parser_for(buffer);

        buffer.push_back(parser->end_delimiter());
        auto checksum_str = parser->checksum();
        buffer.insert(buffer.end(), checksum_str.begin(), checksum_str.end());
        auto msg_delimiter = parser->message_delimiter();
        buffer.insert(buffer.end(), msg_delimiter.begin(), msg_delimiter.end());

        return *this;
    }


    Message& Message::append(Message_buffer&& data)
    {
        // Take ownership of the incoming buffer
        // to maintain move semantics;
        //
        Message_buffer to_append { std::move(data) };
        append(to_append);

        return *this;
    }


    Message& Message::append(std::uint8_t byte)
    {
        return append(Message_buffer { byte });
    }


    Message& Message::append(const std::string& str)
    {
        append(Message_buffer { str.begin(), str.end() });

        return *this;
    }


    void Message::replace(const Message_buffer& src)
    {
        buffer = src;
        parser = Parser::get_parser_for(buffer);
    }


    void Message::replace(Message_buffer&& src)
    {
        buffer = std::move(src);
        parser = Parser::get_parser_for(buffer);
    }


    void Message::replace(Const_iterator src_start, std::size_t src_sz)
    {
        buffer.clear();
        buffer.resize(src_sz);
        std::memcpy(buffer.data(), src_start, src_sz);
        parser = Parser::get_parser_for(buffer);
    }


    void Message::replace(const std::string& src_str)
    {
        replace(Message_buffer { src_str.begin(), src_str.end() });
        parser = Parser::get_parser_for(buffer);
    }


    // -----------------------------------------------------------------------------------------------
    // Data retreival
    //
    Message_buffer Message::relinquish()
    {
        return std::move(buffer);
    }
    

    std::string Message::to_string() const
    {
        return { begin(), end() };
    }


    Message_buffer Message::to_vector() const
    {
        return { begin(), end() };
    }


    std::string Message::payload_as_string() const
    {
        return { payload_begin(), payload_end() };
    }


    Message_buffer Message::payload_as_vector() const
    {
        return { payload_begin(), payload_end() };
    }


    // -----------------------------------------------------------------------------------------------
    // Validity checking
    //
    bool Message::is_valid() const
    {
        if (!parser) return { };
        
        return parser->is_valid();
    }


    bool Message::is_complete() const
    {
        if (!parser) return { };
        
        return parser->is_complete();
    }

} // namespace Navtech::Networking::NMEA_protocol