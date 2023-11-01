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
#include <array>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <new>

#include "Cat240_message.h"
#include "Cat240_video_message.h"
#include "net_conversion.h"
#include "Cat240_datetime.h"
#include "Cat240_extended_info.h"


namespace Navtech::Networking::Cat240_protocol {

    Message::Message() : Message { Type::invalid }
    {
        buffer.resize(Header::size());
        new(buffer.data()) Header { };
    }


    Message::Message(Type type)
    {
        // Construct Header directly into buffer
        //
        buffer.resize(Header::size());
        new(buffer.data()) Header { type };
    }


    Message::Message(const Networking::IP_address& ip_addr, Message::ID id) :
        address         { ip_addr },
        identity        { id }
    {
        buffer.resize(Header::size());
        new(buffer.data()) Header { };
    }


    Message::Message(const Message::Buffer& message)
    {
        replace(message);
    }


    Message::Message(Message::Buffer&& message)
    {
        replace(std::move(message));
    }


    Message::Message(Const_iterator message_start, std::size_t message_sz)
    {
        replace(message_start, message_sz);
    }

    
    Message::Message(Message::ID id, const Message::Buffer& message) :
        identity { id }
    {
        replace(message);
    }


    Message::Message(Message::ID id, Message::Buffer&& message) :
        identity { id }
    {
        replace(std::move(message));
    }


    Message::Message(ID id, Const_iterator message_start, std::size_t message_sz) :
        identity { id }
    {
        replace(message_start, message_sz);
    }
    

    Message::ID Message::client_id() const
    {
        return identity;
    }


    void Message::client_id(Message::ID new_id)
    {
        identity = new_id;
    }


    Type Message::type() const
    {
        auto header = Header::overlay_at(buffer.data());
        return header->type();
    }


    void Message::type(Type t)
    {
        auto header = Header::overlay_at(buffer.data());
        header->type(t);
    }


    const Networking::IP_address& Message::ip_address() const
    {
        return address;
    }
    

    void Message::ip_address(const Networking::IP_address& ip_addr)
    {
        address = ip_addr;
    }


    bool Message::is_valid() const
    {
        if (buffer.empty()) return false;

        auto header = Header::overlay_at(buffer.data());
        return header->is_valid();
    }


    std::size_t Message::size() const
    {
        return buffer.size();
    }


    std::size_t Message::payload_size() const
    {
        if (buffer.empty()) return 0;

        auto header = Header::overlay_at(buffer.data());
        return header->message_length() - Header::size();
    }


    Message::Iterator Message::payload_begin()
    {
        return buffer.data() + Header::size();
    }


    Message::Const_iterator Message::payload_begin() const
    {
        return buffer.data() + Header::size();
    }


    Message::Iterator Message::payload_end()
    {
        return &(*buffer.end());
    }


    Message::Const_iterator Message::payload_end() const
    {
        return &(*buffer.cend());
    }
    

    void  Message::replace(const std::vector<std::uint8_t>& src)
    {
        buffer = src;
    }


    void Message::replace(std::vector<std::uint8_t>&& src)
    {
        buffer = std::move(src);
    }
        
    
    void Message::replace(Message::Const_iterator src_start, std::size_t src_size)
    {
        using std::copy_n;
        using std::back_inserter;

        buffer.clear();
        buffer.reserve(src_size);
        copy_n(src_start, src_size, back_inserter(buffer));
    }


    Message& Message::append(const Message_buffer& data)
    {
        // NB - this code assumes a header has been
        //      previously appended.

        Message_buffer to_insert { data };
        auto aligned_payload_sz = update_payload_info(data);
        to_insert.resize(aligned_payload_sz);
        buffer.insert(buffer.end(), to_insert.begin(), to_insert.end());
        update_message_length();

        return *this;    
    }


    Message& Message::append(Message_buffer&& data)
    {
        Message_buffer to_insert { std::move(data) };

        auto aligned_payload_sz = update_payload_info(data);
        
        to_insert.resize(aligned_payload_sz);
        buffer.insert(buffer.end(), to_insert.begin(), to_insert.end());
        update_message_length();
        
        return *this;    
    }


    Message& Message::append(const Time_of_day& ToD)
    {
        buffer << ToD;
        update_message_length();

        auto header = Header::overlay_at(buffer.data());
        auto flags  = header->features();
        flags.time_of_day = 1;
        header->features(flags);

        return *this;
    }
    
    
    Message& Message::append(Time_of_day&& ToD)
    {
        buffer << std::move(ToD);
        update_message_length();

        auto header = Header::overlay_at(buffer.data());
        auto flags  = header->features();
        flags.time_of_day = 1;
        header->features(flags);

        return *this;
    }


    Message& Message::append(const Extended_info& cfg)
    {
        buffer << cfg;
        update_message_length();

        auto header = Header::overlay_at(buffer.data());
        auto flags  = header->features();
        flags.special_purpose_field = 1;
        header->features(flags);

        return *this;
    }
    
    
    Message& Message::append(Extended_info&& cfg)
    {
        buffer << std::move(cfg);
        update_message_length();

        auto header = Header::overlay_at(buffer.data());
        auto flags  = header->features();
        flags.special_purpose_field = 1;
        header->features(flags);

        return *this;
    }


    std::size_t Message::update_payload_info(const Message_buffer& buffer)
    {
        std::size_t adjusted_sz { };

        switch (type()) {
        case Type::video:
        { 
            auto fft_msg = Video::overlay_at(begin());
            adjusted_sz  = fft_msg->update_video_info(buffer.size());
            break;
        }

        default:
            adjusted_sz = buffer.size();
            break;
        }

        return adjusted_sz;
    }


    void Message::update_message_length()
    {
        auto header = Header::overlay_at(buffer.data());
        header->message_length(buffer.size());
    }


    std::vector<std::uint8_t> Message::relinquish()
    {
        return std::move(buffer);
    }


    Message::Iterator Message::begin()
    {
        return &(*buffer.begin());
    }


    Message::Const_iterator Message::begin() const
    {
        return &(*buffer.begin());
    }


    Message::Iterator Message::end()
    {
        return &(*buffer.end());
    }


    Message::Const_iterator Message::end() const
    {
        return &(*buffer.end());
    }


    Buffer_view Message::data_view() const
    {
        return Buffer_view { buffer };
    }

} // namespace Navtech::Networking::CP_protocol