#include <cstring>
#include <sstream>
#include <iomanip>

#include "NMEA_parser.h"

namespace Navtech::Networking::NMEA_protocol {

    // ------------------------------------------------------------------------
    // Parser
    //

    shared_owner<Parser> Parser::get_parser_for(Message_buffer& buffer)
    {
        std::string msg_str { buffer.begin(), buffer.end() };

        if (msg_str.find("GPRMC") != std::string::npos) return allocate_shared<Basic_parser>(buffer);
        if (msg_str.find("GPHDT") != std::string::npos) return allocate_shared<Basic_parser>(buffer);
        if (msg_str.find("GPGGA") != std::string::npos) return allocate_shared<Basic_parser>(buffer);
        if (msg_str.find("PASHR") != std::string::npos) return allocate_shared<Basic_parser>(buffer);

        if (msg_str.find("BESTPOS") != std::string::npos) return allocate_shared<BESTPOS_parser>(buffer);
        
        return nullptr;
    }

    Parser::Parser(Message_buffer& msg_buffer) :
        buffer { associate_with(msg_buffer) }
    {
    }


    bool Parser::is_complete() const 
    {
        if (buffer->size() < 2)           return false;
        if (*(buffer->end() - 1) != '\n') return false;
        if (*(buffer->end() - 2) != '\r') return false;
        return true;
    }


    Parser::Iterator Parser::begin()
    {
        return &(*buffer->begin());
    }


    Parser::Const_iterator Parser::begin() const
    {
        return &(*buffer->cbegin());
    }


    Parser::Iterator Parser::end()
    {
        // Ignore the message terminator
        //
        return &(*(buffer->end())) - std::strlen(message_terminator);
    }


    Parser::Const_iterator Parser::end() const
    {
        // Ignore the message terminator
        //
        return &(*(buffer->cend())) - std::strlen(message_terminator);
    }


    std::string Parser::message_delimiter() const
    {
        return { message_terminator };
    }


    void Parser::rebind(Message_buffer& new_buffer)
    {
        buffer = associate_with(new_buffer);
    }


    // ------------------------------------------------------------------------
    // Basic_parser
    //
    std::uint8_t Basic_parser::start_delimiter() const
    {
        return start_sentinal;
    }


    std::uint8_t Basic_parser::end_delimiter() const
    {
        return end_sentinal;
    }


    std::string Basic_parser::type_as_string() const
    {
        std::string full_str { payload_begin(), payload_end() };
        return full_str.substr(0, std::strlen(type_format));
    }


    bool Basic_parser::is_valid() const      
    {
        if (*begin() != start_sentinal)        return false;
        if (*payload_end() != end_sentinal)    return false;

        std::string checksum_str  { payload_end() + 1, end() };
        std::string calc_checksum { checksum() };
        return (checksum_str == calc_checksum);
    }


    std::string Basic_parser::checksum() const
    {
        std::uint8_t calculated_checksum { };

        for (Const_iterator itr { payload_begin() }; itr != payload_end(); ++itr) {
            calculated_checksum ^= static_cast<std::uint8_t>(*itr);
        }

        std::stringstream stream { };
        
        stream << std::hex;
        stream << std::setw(2);
        stream << std::setfill('0');
        stream << static_cast<int>(calculated_checksum);
        
        return stream.str();
    }


    Basic_parser::Iterator Basic_parser::payload_begin()      
    {
        return begin() + 1;
    }


    Basic_parser::Const_iterator Basic_parser::payload_begin() const
    {
        return begin() + 1;
    }   


    Basic_parser::Iterator Basic_parser::payload_end()        
    {
        // A basic NMEA message uses an 8-bit checksum
        // stored as two characters.
        // Delimiter            - 1 char
        // Checksum             - 2 char
        //
         return (end() - (sizeof(end_sentinal) + std::strlen(checksum_format)) );
    }


    Basic_parser::Const_iterator Basic_parser::payload_end() const  
    {
        // Delimiter            - 1 char
        // Checksum             - 2 char
        //
         return (end() - (sizeof(end_sentinal) + std::strlen(checksum_format)) );
    }



    // ------------------------------------------------------------------------
    // Extended_parser
    //
    std::uint8_t BESTPOS_parser::start_delimiter() const
    {
        return start_sentinal;
    }


    std::uint8_t BESTPOS_parser::end_delimiter() const
    {
        return end_sentinal;
    }


    std::string BESTPOS_parser::type_as_string() const
    {
        std::string full_str { payload_begin(), payload_end() };
        return full_str.substr(0, std::strlen(type_format));
    }


    bool BESTPOS_parser::is_valid() const      
    {
        if (!is_complete())                    return false;
        if (*begin() != start_sentinal)        return false;
        if (*payload_end() != end_sentinal)    return false;

        std::string checksum_str  { payload_end() + 1, end() };
        std::string calc_checksum { checksum() };
        return (checksum_str == calc_checksum);
    }


    std::string BESTPOS_parser::checksum() const    {
        std::uint32_t calculated_checksum { };
        
        calculated_checksum = Utility::CRC(payload_begin(), payload_end());
        std::stringstream stream { };
        stream << std::hex;
        stream << std::setw(8);
        stream << std::setfill('0');
        stream << calculated_checksum;
        return stream.str();
    }


    BESTPOS_parser::Iterator BESTPOS_parser::payload_begin()      
    {
        return begin() + 1;
    }


    BESTPOS_parser::Const_iterator BESTPOS_parser::payload_begin() const
    {
        return begin() + 1;
    }   


    BESTPOS_parser::Iterator BESTPOS_parser::payload_end()        
    {
        // A basic NMEA message uses an 8-bit checksum
        // stored as two characters.
        // Delimiter            - 1 char
        // Checksum             - 8 char
        //
        return (end() - (sizeof(end_sentinal) + std::strlen(checksum_format)) );
    }


    BESTPOS_parser::Const_iterator BESTPOS_parser::payload_end() const  
    {
        // Delimiter            - 1 char
        // Checksum             - 8 char
        //
        return (end() - (sizeof(end_sentinal) + std::strlen(checksum_format)) );
    }  

} // namespace Navtech::Networking::NMEA_protocol