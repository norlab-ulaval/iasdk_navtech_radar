#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <string>
#include "Message_buffer.h"
#include "pointer_types.h"
#include <numeric>
#include <algorithm>
#include "CRC_generator.h"

namespace Navtech::Networking::NMEA_protocol {
    
    // Abstract Base Class for message interpretation
    //
    class Parser {
    public:
        using ID             = std::uint32_t;
        using Iterator       = std::uint8_t*;
        using Const_iterator = const std::uint8_t*;

        static shared_owner<Parser> get_parser_for(Message_buffer& buffer);

        Parser(Message_buffer& buffer);
        virtual ~Parser() = default;

        virtual std::string     type_as_string() const = 0;

        virtual bool            is_valid() const = 0;
        virtual bool            is_complete() const;
        virtual std::string     checksum() const = 0;

        virtual std::string     message_delimiter() const;
        virtual std::uint8_t    start_delimiter() const = 0;
        virtual std::uint8_t    end_delimiter()   const = 0;

        // payload_begin/payload_end select the data of the message
        // excluding the opening delimiter, closing delimiter and checksum
        // characters.
        //
        virtual Iterator        payload_begin()       = 0;
        virtual Const_iterator  payload_begin() const = 0;
        virtual Iterator        payload_end()         = 0;
        virtual Const_iterator  payload_end() const   = 0;

        // begin/end select the entire message buffer, including
        // message start/end delimiters and checksum.
        //
        Iterator        begin();
        Const_iterator  begin() const;
        Iterator        end();
        Const_iterator  end() const;

        void rebind(Message_buffer& new_buffer);

    private:
        association_to<Message_buffer> buffer;

        static constexpr const char*  message_terminator { "\r\n" }; // CRLF
    };


    // -----------------------------------------------------------------------------------------------
    // Message format:
    //
    //   begin()                       end()
    //   |                             |
    //   v                             v
    //  [$XXXXX .... uint8_t[N] ... *CC\r\n]
    //    ^                         ^
    //    |                         |
    //    payload_begin()           payload_end()
    //
    // where:
    // - XXXXX - message type, stored as five characters
    // - CC    - checksum in hex, stored as two characters
    // - \r\n  - CRLF; two characters
    //
    // -----------------------------------------------------------------------------------------------
    
    class Basic_parser : public Parser {
    public:
        using Parser::Parser;
        using Parser::Iterator;
        using Parser::Const_iterator;

        std::string     type_as_string() const override;

        bool            is_valid() const        override;
        std::string     checksum() const        override;

        std::uint8_t    start_delimiter() const override;
        std::uint8_t    end_delimiter()   const override;

        Iterator        payload_begin()         override;
        Const_iterator  payload_begin() const   override;
        Iterator        payload_end()           override;
        Const_iterator  payload_end() const     override;
    

    private:
        static constexpr const char*  type_format       { "XXXXX" };
        static constexpr const char*  checksum_format   { "CC" };
        static constexpr std::uint8_t start_sentinal    { '$' };
        static constexpr std::uint8_t end_sentinal      { '*' };
    };


    // -----------------------------------------------------------------------------------------------
    // Message format:
    //
    //   begin()                               end()
    //   |                                     |
    //   v                                     v
    //  [$XXXXXXX .... uint8_t[N] ... *CCCCCCCC\r\n]
    //    ^                           ^
    //    |                           |
    //    payload_begin()             payload_end()
    //
    // where:
    // - XXXXXXX    - Message type string, as seven characters
    // - CCCCCCCC   - checksum in hex, stored as eight characters
    // - \r\n       - CRLF; two characters
    //
    // -----------------------------------------------------------------------------------------------
    
    class BESTPOS_parser : public Parser {
    public:
        public:
        using Parser::Parser;
        using Parser::Iterator;
        using Parser::Const_iterator;

        std::string     type_as_string() const override;

        bool            is_valid() const        override;
        std::string     checksum() const        override;

        std::uint8_t    start_delimiter() const override;
        std::uint8_t    end_delimiter()   const override;

        Iterator        payload_begin()         override;
        Const_iterator  payload_begin() const   override;
        Iterator        payload_end()           override;
        Const_iterator  payload_end() const     override;
        
    private:
        static constexpr const char*  type_format       { "XXXXXXX" };
        static constexpr const char*  checksum_format   { "CCCCCCCC" }; 
        static constexpr std::uint8_t start_sentinal    { '#' };
        static constexpr std::uint8_t end_sentinal      { '*' };
    };

} // namespace Navtech::Networking::NMEA_protocol


#endif // NMEA_PARSER_H