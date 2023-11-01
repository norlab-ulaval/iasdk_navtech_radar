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

#ifndef NMEA_MESSAGE_H
#define NMEA_MESSAGE_H

#include <cstdint>
#include <vector>
#include <string_view>

#include "pointer_types.h"
#include "Message_buffer.h"
#include "Time_utils.h"
#include "NMEA_message_types.h"
#include "NMEA_parser.h"

namespace Navtech::Networking::NMEA_protocol {


    class Message {
    public:
        using ID             = std::uint32_t;
        using Iterator       = std::uint8_t*;
        using Const_iterator = const std::uint8_t*;

        // Constructors
        //
        Message() = default;
        Message(const Message_buffer& message_vector);
        Message(Message_buffer&& message_vector);
        Message(Const_iterator message_start, std::size_t message_sz);
        Message(std::string_view msg_str);

        // Copy/move policy
        //
        Message(const Message& other);
        Message(Message&& other);
        Message& operator=(const Message& rhs);
        Message& operator=(Message&& rhs);

        void client_id(ID id);
        ID client_id() const;

        Type type() const;
        void type(Type t);
        std::string type_as_string() const;

        bool is_valid() const;
        bool is_complete() const;

        // size() = header_size() + payload_size()
        //
        std::size_t size() const;
        std::size_t payload_size() const;

        // More the entire contents of the message
        // into a message buffer. If there is a time
        // stamp set, this will be preprended to the
        // buffer.
        //
        // NOTE:
        // This is the ONLY function that will add 
        // the timestamp!
        //
        Message_buffer relinquish();

        // begin/end select the entire message buffer, including
        // message start/end delimiters and checksum.
        //
        Iterator        begin();
        Const_iterator  begin() const;
        Iterator        end();
        Const_iterator  end() const;

        Buffer_view data_view() const;

        // Full message, including delimiters, but not checksum
        //
        std::string     to_string() const;
        Message_buffer  to_vector() const;

        // Message without delimiters
        //
        std::string     payload_as_string() const;
        Message_buffer  payload_as_vector() const;

        // payload_begin/payload_end select the data of the message
        // excluding the opening delimiter, closing delimiter and checksum
        // characters.
        //
        Iterator        payload_begin();
        Const_iterator  payload_begin() const;
        Iterator        payload_end();
        Const_iterator  payload_end() const;

        // Data insertion
        //
        Message& append(const Message_buffer& data);
        Message& append(Message_buffer&& data);
        Message& append(const std::string& str);
        Message& append(std::uint8_t byte);

        // Replace or retrieve the entire data contents of the message.
        // These functions will invalidate any iterators.
        //
        void replace(const Message_buffer& src);
        void replace(Message_buffer&& src);
        void replace(Const_iterator src_start, std::size_t src_sz);
        void replace(const std::string& src_str);
        
    protected:
        // std::uint8_t checksum() const;

    private:
        Message_buffer buffer   { '$', '*', '0', '0', '\r', '\n' };
        ID             identity { };

        shared_owner<Parser> parser { };
    };

    using Pointer = shared_owner<Message>;

} // namespace Navtech::Networking::NMEA_protocol

#endif // NMEA_MESSAGE_H