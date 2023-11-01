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

#ifndef COLOSSUS_NETWORK_MESSAGE_H
#define COLOSSUS_NETWORK_MESSAGE_H

#include <cstdint>
#include <vector>
#include <cstring>

#include "Colossus_message_types.h"
#include "IP_address.h"
#include "pointer_types.h"
#include "Message_buffer.h"

namespace Navtech::Networking::Colossus_protocol {

    // --------------------------------------------------------------------------------------------------
    // Signature is a helper class for message signatures
    //
    class Signature {
    public:
        using Array          = std::array<std::uint8_t, 16>;
        using Const_iterator = const uint8_t*;

        Const_iterator begin() const;
        Const_iterator end()   const;

        std::size_t size() const;

        std::vector<std::uint8_t> to_vector() const;
        std::string               to_string() const;

    private:
        static const Array signature;
    };

    bool operator==(const Signature& lhs, const std::vector<std::uint8_t>& rhs);
    bool operator==(const std::vector<std::uint8_t>& lhs, const Signature& rhs);
    bool operator!=(const Signature& lhs, const std::vector<std::uint8_t>& rhs);
    bool operator!=(const std::vector<std::uint8_t>& lhs, const Signature& rhs);


    // --------------------------------------------------------------------------------------------------
    // The Message class provides an interface for storing and accessing
    // CP Networking messages.
    //
    class Message {
    public:
        using ID             = std::uint32_t;
        using Iterator       = std::uint8_t*;
        using Const_iterator = const std::uint8_t*;
        using Buffer         = std::vector<std::uint8_t>;
    

        // Constructors
        //
        Message();
        Message(Type msg_type);
        Message(const Networking::IP_address& ip_addr, ID id);
        
        Message(const Buffer& message_vector);
        Message(Buffer&& message_vector);
        Message(Const_iterator message_start, std::size_t message_sz);
        
        Message(ID id);
        Message(ID id, const Buffer& message);
        Message(ID id, Buffer&& message);
        Message(ID id, Const_iterator message_start, std::size_t message_sz);

        // Colossus message interface
        //
        ID   client_id() const;
        void client_id(ID new_id);

        Type type() const;
        void type(Type t);

        const Networking::IP_address& ip_address() const;
        void  ip_address(const Networking::IP_address& ip_addr);

        bool  is_valid() const;

        // size() = header_size() + payload_size()
        //
        std::size_t size() const;
        std::size_t payload_size() const;
        static constexpr std::size_t header_size() { return sizeof(Header); }

        // Access to a valid signature
        //
        static const Signature& valid_signature();

        // Replace or retrieve the entire data contents of the message.
        // These functions will invalidate any views.
        //
        void replace(const Buffer& src);
        void replace(Buffer&& src);
        void replace(Const_iterator src_start, std::size_t src_sz);
        
        // Data retrieval
        //
        std::vector<std::uint8_t> relinquish();

        Iterator        begin();
        Const_iterator  begin() const;
        Iterator        end();
        Const_iterator  end() const;

        Buffer_view data_view() const;

        // Add a protocol buffer of data to the message
        // These functions will invalidate any views.
        //
        Message& append(const Buffer& protocol_buffer);
        Message& append(Buffer&& protocol_buffer);
        Message& append(const std::string& protocol_buffer);
        Message& append(std::string&& protocol_buffer);

        Message& operator<<(const Buffer& protocol_buffer);
        Message& operator<<(Buffer&& protocol_buffer);
        Message& operator<<(const std::string& protocol_buffer);
        Message& operator<<(std::string&& protocol_buffer);

        // Add a header.  The header is always inserted before any
        // protocol buffer.  In general, prefer to add the header
        // *before* adding the protocol buffer.
        // This function will invalidate any views.
        //
        template <typename Header_Ty>
        Message& append(const Header_Ty& header);

        template <typename Header_Ty>
        Message& operator<<(const Header_Ty& header);

        // Interpret the message contents as the provided message type.
        // Pre-conditions:
        // - Message is valid
        // - Message type is correct (matches return from type())
        //
        template <typename Header_Ty>
        Header_Ty* view_as();

        template <typename Header_Ty>
        const Header_Ty* view_as() const;

    protected:
        void initialize();
        
        void           update_payload_size();
        Iterator       payload_begin();
        Const_iterator payload_begin() const;
        Iterator       payload_end();
        Const_iterator payload_end() const;

        void display();

        bool           is_signature_valid() const;
        void           add_signature();
        Iterator       signature_begin();
        Const_iterator signature_begin() const;
        Iterator       signature_end();
        Const_iterator signature_end() const;

        bool           is_version_valid() const;
        void           add_version(std::uint8_t version);

    private:
        // Header is for overlay only - header information is stored in 
        // data vector, contiguous with the payload.
        //
        #pragma pack(1)   
        struct Header {
            static constexpr std::size_t signature_sz { 16 };

            std::uint8_t  signature[signature_sz];
            std::uint8_t  version;
            Type          id;
            std::uint32_t payload_size;

            static Header* overlay_onto(std::uint8_t* from)             { return reinterpret_cast<Header*>(from); }
            static const Header* overlay_onto(const std::uint8_t* from) { return reinterpret_cast<const Header*>(from); }
        };
        #pragma pack() 

        static_assert(sizeof(Message::Header) == 22);

        Networking::IP_address address      { };
        ID                  identity     { };
        bool                has_protobuf { };
        Buffer              data         { };
    };

    // -------------------------------------------------------

    using Pointer = shared_owner<Message>;


    // -------------------------------------------------------

    template <typename Header_Ty>
    Message& Message::append(const Header_Ty& header)
    {
        using std::begin;
        using std::end;
        using std::copy_backward;

        data.resize(size() + header.header_size());
        
        if (has_protobuf) {
            auto start  = payload_begin();
            auto finish = start + payload_size();
            copy_backward(start, finish, end(data));
        }

        // std::copy(header.begin(), header.end(), payload_begin());
        
        std::memcpy(payload_begin(), header.begin(), header.size());

        update_payload_size();
        
        return *this;
    }


    template <typename Header_Ty>
    Message& Message::operator<<(const Header_Ty& header)
    {
        return append(header);
    }


    template <typename Header_Ty>
    Header_Ty* Message::view_as()
    {
        return reinterpret_cast<Header_Ty*>(data.data());
    }
    

    template <typename Header_Ty>
    const Header_Ty* Message::view_as() const
    {
        return reinterpret_cast<const Header_Ty*>(data.data());
    }

} // namespace Navtech::Networking::Colossus_protocol

#endif // COLOSSUS_NETWORK_MESSAGE_H