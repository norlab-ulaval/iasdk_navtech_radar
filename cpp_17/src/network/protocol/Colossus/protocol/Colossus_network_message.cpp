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

#ifdef __linux
#include <arpa/inet.h>
#else
#include "winsock.h"
#endif

#include "Colossus_network_message.h"

// Constants
//
constexpr unsigned int largest_valid_message { 255 };
constexpr unsigned int largest_payload       { 1'000'000 };
constexpr std::uint8_t version               { 1 };


namespace Navtech::Networking::Colossus_protocol {

    // ---------------------------------------------------------------------------------------------------------
    // Signature
    //
    const Signature::Array Signature::signature { 
        0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0F, 0x0F, 
        0x1F, 0x1F, 0x3F, 0x3F, 0x7F, 0x7F, 0xFE, 0xFE
    };

            
    Signature::Const_iterator Signature::begin() const
    {
        return &signature[0];
    }
            
    
    Signature::Const_iterator Signature::end() const
    {
        return &signature[16];
    }


    std::size_t Signature::size() const
    {
        return signature.size();
    }

    
    std::vector<std::uint8_t> Signature::to_vector() const
    {
        return std::vector<std::uint8_t> { begin(), end() };
    }


    std::string Signature::to_string() const
    {
        std::ostringstream os { };

        os << std::hex;

        for (unsigned i { 0 }; i < signature.size() - 1; ++i) {
            os  << "0x"
                << std::setw(2)
                << std::setfill('0')
                << static_cast<int>(signature[i]) << ", ";
        }
        os  << "0x"
            << std::setw(2)
            << std::setfill('0')
            << static_cast<int>(signature[signature.size() - 1]);

        return os.str();
    }


    bool operator==(const Signature& lhs, const std::vector<std::uint8_t>& rhs)
    {
        return equal(
            rhs.cbegin(),
            rhs.cend(),
            lhs.begin()
        );
    }


    bool operator==(const std::vector<std::uint8_t>& lhs, const Signature& rhs)
    {
        return operator==(rhs, lhs);
    }
    
    
    bool operator!=(const Signature& lhs, const std::vector<std::uint8_t>& rhs)
    {
        return !(lhs == rhs);
    }
        
        
    bool operator!=(const std::vector<std::uint8_t>& lhs, const Signature& rhs)
    {
        return !(lhs == rhs);
    }


    // ---------------------------------------------------------------------------------------------------------
    // Message
    //
    Message::Message()
    {
        initialize();
    }


    Message::Message(Type msg_type)
    {
        initialize();
        type(msg_type);
    }


    Message::Message(Message::ID id) :
        identity { id }
    {
        initialize();
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
        auto header = Header::overlay_onto(data.data());
        return header->id;
    }


    void Message::type(Type t)
    {
        auto header = Header::overlay_onto(data.data());
        header->id = t;
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
        if (data.empty()) return false;

        return (
            is_signature_valid()                                  &&
            is_version_valid()                                    &&
            static_cast<unsigned>(type()) <= largest_valid_message &&
            payload_size() < largest_payload
        );
    }


    std::size_t Message::size() const
    {
        return data.size();
    }


    const Signature& Message::valid_signature()
    {
        static Signature signature { };
        return signature;
    }


    std::size_t Message::payload_size() const
    {
        if (data.empty()) return 0;

        auto header = Header::overlay_onto(data.data());
        return ntohl(header->payload_size);
    }


    void Message::update_payload_size()
    {
        auto header = Header::overlay_onto(data.data());
        header->payload_size = htonl(data.size() - header_size());
    }


    Message::Iterator Message::payload_begin()
    {
        return data.data() + header_size();
    }


    Message::Const_iterator Message::payload_begin() const
    {
        return data.data() + header_size();
    }


    Message::Iterator Message::payload_end()
    {
        return &(*data.end());
    }


    Message::Const_iterator Message::payload_end() const
    {
        return &(*data.cend());
    }
    

    void  Message::replace(const std::vector<std::uint8_t>& src)
    {
        data = src;
    }


    void Message::replace(std::vector<std::uint8_t>&& src)
    {
        using std::move;

        data = move(src);
    }
        
    
    void Message::replace(Message::Const_iterator src_start, std::size_t src_size)
    {
        using std::copy_n;
        using std::back_inserter;

        data.clear();
        data.reserve(src_size);
        copy_n(src_start, src_size, back_inserter(data));
    }


    std::vector<std::uint8_t> Message::relinquish()
    {
        using std::move;
        using std::vector;
        using std::uint8_t;

        // add_signature();
        return vector<uint8_t> { move(data) };
    }


    Message::Iterator Message::begin()
    {
        return &(*data.begin());
    }


    Message::Const_iterator Message::begin() const
    {
        return &(*data.begin());
    }


    Message::Iterator Message::end()
    {
        return &(*data.end());
    }


    Message::Const_iterator Message::end() const
    {
        return &(*data.end());
    }


    Buffer_view Message::data_view() const
    {
        return Buffer_view { data };
    }

    
    Message& Message::append(const Message::Buffer& protocol_buffer)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        // data.reserve(data.size() + protocol_buffer.size());
        // copy(begin(protocol_buffer), end(protocol_buffer), back_inserter(data));

        auto protobuf_offset = data.size();

        data.resize(data.size() + protocol_buffer.size());
        std::memcpy(
            data.data() + protobuf_offset,
            protocol_buffer.data(),
            protocol_buffer.size()
        );
       
        update_payload_size();
        has_protobuf = true;
        return *this;
    }


    Message& Message::append(Message::Buffer&& protocol_buffer)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        // Take ownership to ensure correct move semantics
        // for client code
        //
        Buffer temp { move(protocol_buffer) };  

        // data.reserve(data.size() + temp.size());
        // copy(begin(temp), end(temp), back_inserter(data));

        auto protobuf_offset = data.size();

        data.resize(data.size() + temp.size());
        std::memcpy(
            data.data() + protobuf_offset,
            temp.data(),
            temp.size()
        );

        update_payload_size();
        has_protobuf = true;
        return *this;
    }


    Message& Message::append(const std::string& protocol_buffer)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        data.reserve(data.size() + protocol_buffer.size());
        copy(begin(protocol_buffer), end(protocol_buffer), back_inserter(data));
       
        update_payload_size();
        has_protobuf = true;
        return *this;
    }



    Message& Message::append(std::string&& protocol_buffer)
    {
        using std::begin;
        using std::end;
        using std::copy;
        using std::back_inserter;

        // Take ownership to ensure correct move semantics
        // for client code
        //
        std::string temp { move(protocol_buffer) };  

        data.reserve(data.size() + temp.size());
        copy(begin(temp), end(temp), back_inserter(data));

        update_payload_size();
        has_protobuf = true;
        return *this;
    }


    Message& Message::operator<<(const Message::Buffer& protocol_buffer)
    {
        return append(protocol_buffer);
    }


    Message& Message::operator<<(Message::Buffer&& protocol_buffer)
    {
        return append(std::move(protocol_buffer));
    }


    Message& Message::operator<<(const std::string& protocol_buffer)
    {
        return append(protocol_buffer);
    }


    Message& Message::operator<<(std::string&& protocol_buffer)
    {
        return append(std::move(protocol_buffer));
    }


    void Message::initialize()
    {
        data.resize(header_size());
        add_signature();
        add_version(version);
    }


    bool Message::is_signature_valid() const
    {
        using std::array;
        using std::begin;
        using std::uint8_t;
        using std::equal;

        return equal(valid_signature().begin(), valid_signature().end(), signature_begin());
    }


    void Message::add_signature()
    {
        using std::begin;
        using std::end;
        using std::copy;

        copy(valid_signature().begin(), valid_signature().end(), signature_begin());
    }


    bool Message::is_version_valid() const
    {
        auto header = Header::overlay_onto(data.data());

        return (header->version == version);
    }


    void Message::add_version(std::uint8_t version)
    {
        auto header = Header::overlay_onto(data.data());

        header->version = version;
    }


    Message::Iterator Message::signature_begin()
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature[0]);
    }


    Message::Const_iterator Message::signature_begin() const
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature[0]);
    }


    Message::Iterator Message::signature_end()
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature[Header::signature_sz]); // One-past-the-end
    }


    Message::Const_iterator Message::signature_end() const
    {
        auto header = Header::overlay_onto(data.data());
        return &(header->signature[Header::signature_sz]);
    }


} // namespace Navtech::Networking::CP_protocol