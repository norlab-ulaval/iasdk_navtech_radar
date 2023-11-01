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

#ifndef CAT240_NETWORK_MESSAGE_H
#define CAT240_NETWORK_MESSAGE_H

#include <cstdint>
#include <vector>
#include <cstring>

#include "Cat240_header.h"
#include "Cat240_message_types.h"
#include "IP_address.h"
#include "Message_buffer.h"

#include "pointer_types.h"


namespace Navtech::Networking::Cat240_protocol {

    class Time_of_day;
    class Extended_info;

    // --------------------------------------------------------------------------------------------------
    // Manages the buffer holding the CAT240 message
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
        
        Message(ID id, const Buffer& message);
        Message(ID id, Buffer&& message);
        Message(ID id, Const_iterator message_start, std::size_t message_sz);

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

        // Add a specific message; and its data
        // This function will invalidate any views.
        //
        template <typename Header_Ty>
        Message& append(const Header_Ty& header);

        template <typename Header_Ty>
        Message& operator<<(const Header_Ty& header);

        Message& append(const Message_buffer& data);
        Message& append(Message_buffer&& data);
        Message& append(const Time_of_day& ToD);
        Message& append(Time_of_day&& ToD);
        Message& append(const Extended_info& cfg);
        Message& append(Extended_info&& cfg);

        // Interpret the message contents as the provided message type.
        // Pre-conditions:
        // - Message is valid
        // - Message type is correct (matches return from type())
        //
        template <typename Message_Ty>
        Message_Ty* view_as();

        template <typename Message_Ty>
        const Message_Ty* view_as() const;

    protected:
        void initialize();
        
        void           update_message_size();
        Iterator       payload_begin();
        Const_iterator payload_begin() const;
        Iterator       payload_end();
        Const_iterator payload_end() const;

        void display();

    private:
        Networking::IP_address address   { };
        ID                  identity     { };
        Buffer              buffer       { };

        std::size_t update_payload_info(const Message_buffer& buffer);
        void update_message_length();
    };


    // -----------------------------------------------------------------------
    //
    using Pointer = shared_owner<Message>;


    // -----------------------------------------------------------------------
    //
    template <typename Header_Ty>
    Message& Message::append(const Header_Ty& header)
    {
        buffer.clear();
        buffer << header;
        update_message_length();
        return *this;
    }


    template <typename Header_Ty>
    Message& Message::operator<<(const Header_Ty& header)
    {
        return append(header);
    }


    template <typename Message_Ty>
    Message_Ty* Message::view_as()
    {
        return reinterpret_cast<Message_Ty*>(begin());
    }
    

    template <typename Message_Ty>
    const Message_Ty* Message::view_as() const
    {
        return reinterpret_cast<const Message_Ty*>(begin());
    }

} // namespace Navtech::Networking::Cat240_protocol

#endif // CAT240_NETWORK_MESSAGE_H