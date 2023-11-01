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

#ifndef CAT240_HEADER_H
#define CAT240_HEADER_H

#include <cstdint>
#include <vector>

#include "Cat240_message_types.h"
#include "net_conversion.h"

#define USE_SPX_VERSION

namespace Navtech::Networking::Cat240_protocol {

// DO NOT REMOVE - 
// This ensures correct alignment for all messages
//
#pragma pack(1)

#ifdef USE_SPX_VERSION
    // Constants
    //
    constexpr std::uint8_t  largest_valid_message { 255 };
    constexpr std::uint8_t  Navtech_SAC           { 0xCA };      // TODO - use proper values
    constexpr std::uint8_t  Navtech_SIC           { 0xFE };      // TODO - use proper values

    // Feature flags - CP's implementation goes in opposite order to
    // CAT-240 table 3
    //   
    struct FSPEC {
        std::uint16_t extended               : 1;    // *Must* be zero 
        std::uint16_t special_purpose_field  : 1;
        std::uint16_t                        : 1;
        std::uint16_t time_of_day            : 1;

        std::uint16_t video_high_volume      : 1;    // " " " "
        std::uint16_t video_medium_volume    : 1;    // Mutually exclusive
        std::uint16_t video_low_volume       : 1;    // " " " "
        std::uint16_t cell_group_size        : 1;

        std::uint16_t extended_fspec         : 1;    // *Must* be set to one
        std::uint16_t resolution             : 1;
        std::uint16_t femto_header           : 1;    // " " " "
        std::uint16_t nano_header            : 1;    // Mutually exclusive

        std::uint16_t video_summary          : 1;
        std::uint16_t video_record_header    : 1;
        std::uint16_t message_type           : 1;
        std::uint16_t data_source_id         : 1;
    };

    struct Data_source_id {
        std::uint8_t SIC;
        std::uint8_t SAC;
    };

    class Header {
    public:
        // Construction
        //
        inline Header();
        inline Header(Type t);

        // Accessors/mutators
        //
        inline Type type() const;
        inline void type(Type t); 

        inline FSPEC features() const;
        inline void  features(FSPEC feature_flags);

        inline static constexpr std::size_t size();

        inline std::size_t message_length() const;
        inline void message_length(std::size_t sz);

        inline Data_source_id id() const;
        inline void id(Data_source_id dsi);

        inline bool is_valid() const;

        // Iterators
        //
        std::uint8_t*       begin()   		{ return reinterpret_cast<std::uint8_t*>(this); }
        const std::uint8_t* begin() const	{ return reinterpret_cast<const std::uint8_t*>(this); }
        std::uint8_t*       end()		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(Header)); }
        const std::uint8_t* end() const     { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(Header)); }

        // Memory overlays
        //
        static Header*       overlay_at(std::uint8_t* addr)       { return reinterpret_cast<Header*>(addr); }
        static const Header* overlay_at(const std::uint8_t* addr) { return reinterpret_cast<const Header*>(addr); }

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.

        // Data Block
        //
        std::uint8_t    category        { 240 };
        std::uint16_t   length          { static_cast<std::uint16_t>(Header::size()) };
        
        union FSPEC_word {
            FSPEC         flags;
            std::uint16_t word;
        };

        std::uint16_t feature_spec      { };
        
        // The following fields are common to both video messages
        // and video summary messages
        
        // I240/010
        //
        union DSI_word {
            Data_source_id data_source;
            std::uint16_t  word;
        };

        std::uint16_t data_source_id;
    
        // I240/000
        //
        Type msg_type { Type::invalid };
    };

#else

    // Constants
    //
    constexpr std::uint8_t  largest_valid_message { 255 };
    constexpr std::uint8_t  Navtech_SAC           { 0xCA };      // TODO - use proper values
    constexpr std::uint8_t  Navtech_SIC           { 0xFE };      // TODO - use proper values

    // Feature flags - indicates the presence of an item in the message. 
    // These fields are defined in Table 3 - Standard UAP
    //   
    struct FSPEC {
        std::uint16_t data_source_id         : 1;
        std::uint16_t message_type           : 1;
        std::uint16_t video_record_header    : 1;
        std::uint16_t video_summary          : 1;
        std::uint16_t nano_header            : 1;    // Mutually exclusive
        std::uint16_t femto_header           : 1;    // " " " "
        std::uint16_t resolution             : 1;
        std::uint16_t extended_fspec         : 1;    // *Must* be set to one        
        std::uint16_t cell_group_size        : 1;
        std::uint16_t video_low_volume       : 1;    // " " " "
        std::uint16_t video_medium_volume    : 1;    // Mutually exclusive
        std::uint16_t video_high_volume      : 1;    // " " " "
        std::uint16_t time_of_day            : 1;
        std::uint16_t                        : 1;
        std::uint16_t special_purpose_field  : 1;
        std::uint16_t extended               : 1;    // *Must* be zero       
    };

    struct Data_source_id {
        std::uint8_t SIC;
        std::uint8_t SAC;
    };

    class Header {
    public:
        // Construction
        //
        inline Header();
        inline Header(Type t);

        // Accessors/mutators
        //
        inline Type type() const;
        inline void type(Type t); 

        inline FSPEC features() const;
        inline void  features(FSPEC feature_flags);

        inline static constexpr std::size_t size();

        inline std::size_t message_length() const;
        inline void message_length(std::size_t sz);

        inline Data_source_id id() const;
        inline void id(Data_source_id dsi);

        inline bool is_valid() const;

        // Iterators
        //
        std::uint8_t*       begin()   		{ return reinterpret_cast<std::uint8_t*>(this); }
        const std::uint8_t* begin() const	{ return reinterpret_cast<const std::uint8_t*>(this); }
        std::uint8_t*       end()		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(Header)); }
        const std::uint8_t* end() const     { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(Header)); }

        // Memory overlays
        //
        static Header*       overlay_at(std::uint8_t* addr)       { return reinterpret_cast<Header*>(addr); }
        static const Header* overlay_at(const std::uint8_t* addr) { return reinterpret_cast<const Header*>(addr); }

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.

        // Data Block
        //
        std::uint8_t    category        { 240 };
        std::uint16_t   length          { static_cast<std::uint16_t>(Header::size()) };
        
        union FSPEC_word {
            FSPEC         flags;
            std::uint16_t word;
        };

        std::uint16_t feature_spec      { };
        
        // The following fields are common to both video messages
        // and video summary messages
        
        // I240/000
        //
        Type msg_type { Type::invalid };

        // I240/010
        //
        union DSI_word {
            Data_source_id data_source;
            std::uint16_t  word;
        };

        std::uint16_t data_source_id;
    };

#endif // USE_SPX_VERSION

    static_assert(sizeof(Header) == 8);

    // -----------------------------------------------------------------------------------------------------------------
    //
    Header::Header() : Header { Type::invalid }
    {
    }


    Header::Header(Type t) : msg_type { t }
    {
        id(Data_source_id { Navtech_SIC, Navtech_SAC });
    }


    Type Header::type() const                   
    { 
        return msg_type; 
    }
    
    
    void Header::type(Type t)                   
    { 
        msg_type = t; 
    }


    FSPEC Header::features() const
    {
        FSPEC_word fspec { };

        fspec.word = to_uint16_host(feature_spec);
        return fspec.flags;
    }


    void Header::features(FSPEC feature_flags)
    {
        FSPEC_word fspec { };

        fspec.flags  = feature_flags;
        feature_spec = to_uint16_network(fspec.word);
    }


    Data_source_id Header::id() const
    {
        DSI_word dsi { };

        dsi.word = to_uint16_host(data_source_id);
        return dsi.data_source;
    }


    void Header::id(Data_source_id source_id)
    {
        DSI_word dsi { };
        dsi.data_source = source_id;
        data_source_id  = to_uint16_network(dsi.word);
    }


    constexpr std::size_t Header::size() { return sizeof(Header); }


    std::size_t Header::message_length() const  
    { 
        return to_uint16_host(length); 
    }


    void Header::message_length(std::size_t sz) 
    { 
        length = to_uint16_network(sz); 
    }


    bool Header::is_valid() const
    {
        return (
            category == 240                                         &&
            msg_type != Type::invalid                               &&
            static_cast<unsigned>(type()) <= largest_valid_message
        );
    }


    inline std::vector<std::uint8_t>& operator<<(std::vector<std::uint8_t>& v, const Header& hdr)
    {
        v.insert(v.end(), hdr.begin(), hdr.end());
        return v;
    }

// DO NOT REMOVE - 
// This ensures correct alignment for all messages
//
#pragma pack()

} // namespace Navtech::Networking::Cat240_protocol

#endif // CAT240_HEADER_H