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

#ifndef CAT240_VIDEO_MESSAGE_H
#define CAT240_VIDEO_MESSAGE_H


#include "Cat240_header.h"
#include "net_conversion.h"
#include "Units.h"
#include "uint24_t.h"

using namespace Navtech::Utility;
using namespace Navtech::Unit;

namespace Navtech::Networking::Cat240_protocol {

// DO NOT REMOVE - 
// This ensures correct alignment for all messages
//
#pragma pack(1)

    class Video : public Header {
    public:
        enum Resolution : std::uint8_t { invalid, monobit, low, medium, high, very_high, ultra_high };

        // Construction
        //
        inline Video();

        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        static constexpr std::size_t size();
        
        inline std::uint32_t sweep_counter() const;
        inline void          sweep_counter(std::uint32_t count);

        // Actual number of bytes in FFT payload
        // video_size <= video_block_size
        // update_video_info() returns video_block_size() for 
        // a supplied FFT data size.
        //
        inline std::size_t video_size() const;
        inline void        video_size(std::size_t sz);
        inline std::size_t update_video_info(std::size_t data_sz);

        // Allocated memory for FFT blocks
        // video_block_size = block_size * block_count
        //
        inline std::size_t block_size() const;
        inline std::size_t block_count() const;
        inline void        block_count(std::size_t count);
        inline std::size_t video_block_size() const;
        inline void        update_block_info(std::size_t);      

        inline Resolution  resolution() const;      
        inline void        resolution(Resolution res);
        inline bool        is_compressed() const;

        // Azimuth information
        //
        inline void        start_angle(const Degrees& start);      
        inline Degrees     start_angle() const;      

        inline void        end_angle(const Degrees& end);      
        inline Degrees     end_angle() const;

        inline void        start_bin(Unit::Bin first);
        inline Unit::Bin   start_bin() const;  
        inline std::size_t num_bins() const;
        inline void        num_bins(std::size_t bins);   
        inline Unit::Metre bin_size() const;      
        inline void        bin_size(Unit::Metre m);      
        
        // Header to vector
        //
        inline std::vector<std::uint8_t> to_vector() const;

        // Iterators
        //
        std::uint8_t*       begin()   		{ return reinterpret_cast<std::uint8_t*>(this); }
        const std::uint8_t* begin() const	{ return reinterpret_cast<const std::uint8_t*>(this); }
        std::uint8_t*       end()		    { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(Video)); }
        const std::uint8_t* end() const     { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(Video)); }

        static Video*       overlay_at(std::uint8_t* addr)       { return reinterpret_cast<Video*>(addr); }
        static const Video* overlay_at(const std::uint8_t* addr) { return reinterpret_cast<const Video*>(addr); }

        // Payload iterators
        // Return iterators to the (padded) video data block
        // Note - these iterators are pointing to data *outside* this object.
        // If there is no data after this header the behaviour is undefined.
        //
        inline std::uint8_t*       video_begin()   		{ return end(); }
        inline const std::uint8_t* video_begin() const	{ return end(); }
        inline std::uint8_t*       video_end()		    { return end() + video_block_size(); }
        inline const std::uint8_t* video_end() const    { return end() + video_block_size(); }
        // FFT data, without padding
        //
        inline std::vector<std::uint8_t> video_to_vector() const;

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.
        
        // I240/020
        //
        std::uint32_t index  { };            

        // I240/041 - fempto header (normally)
        //
        std::uint16_t start_azimuth { };    
        std::uint16_t end_azimuth   { };
        std::uint32_t range_start   { };
        std::uint32_t cell_duration { };

        // I240/048
        //
        struct Compression {
            std::uint8_t            : 7;
            std::uint8_t compressed : 1;
        };

        struct I240_048 {
            Resolution  resolution;
            Compression compression;
        };

        union Resolution_compression {
            I240_048        field;
            std::uint16_t   word;
        };

        std::uint16_t cell_resolution { };

        // I240/049
        //
        std::uint16_t   video_size_bytes { };
        uint24_t        video_size_cells { };

        // I240/050 - low volume.   Block size: 4
        // I240/050 - med volume.   Block size: 64
        // I240/050 - high volume.  Block size: 256 
        //
        std::uint8_t num_blocks { };

        // Helper functions
        //
        inline           int           bit_resolution() const;
        inline constexpr std::uint32_t to_fsec(Unit::Metre s) const;
        inline constexpr Unit::Metre   to_metre(std::uint32_t dur_fsec) const;
        inline constexpr std::uint16_t angle_to_uint16(const Unit::Degrees& angle) const;
        inline constexpr Unit::Degrees to_angle(std::uint16_t value) const;
    };

    static_assert(sizeof(Video) == 32);

    // -----------------------------------------------------------------------------------------------------------------
    //
    Video::Video() : Header { Type::video }
    {
        FSPEC flags { };

        flags.data_source_id        = 1;
        flags.message_type          = 1;
        flags.video_record_header   = 1;
        flags.femto_header          = 1;
        flags.resolution            = 1;
        flags.extended_fspec        = 1;
        flags.cell_group_size       = 1;

        features(flags);
        message_length(sizeof(Video));
        resolution(Resolution::high);
    }


    constexpr std::size_t Video::size() 
    { 
        return sizeof(Video); 
    }


    std::vector<std::uint8_t> Video::to_vector() const
    {    
        return { begin(), end() }; 
    }


    std::uint32_t Video::sweep_counter() const         
    { 
        return to_uint32_host(index); 
    }


    void Video::sweep_counter(std::uint32_t count)     
    { 
        index = to_uint32_network(count); 
    }


    std::size_t Video::block_count() const             
    { 
        return num_blocks; 
    }


    void Video::block_count(std::size_t count)
    {
        num_blocks = count;
    }

    
    Video::Resolution Video::resolution() const               
    {
        Resolution_compression res { };
        
        res.word = to_uint16_host(cell_resolution);
        return res.field.resolution;
    }


    void Video::resolution(Video::Resolution res)             
    { 
        Resolution_compression r { };
        r.field.resolution = res;
        cell_resolution = to_uint16_network(r.word);
    }


    bool Video::is_compressed() const
    {
        return false;
    }


    void Video::start_angle(const Degrees& start)      
    { 
        start_azimuth = to_uint16_network(angle_to_uint16(start)); 
    }


    Degrees Video::start_angle() const                 
    { 
        return to_angle(to_uint16_host(start_azimuth)); 
    }


    void Video::end_angle(const Degrees& end)          
    { 
        end_azimuth = to_uint16_network(angle_to_uint16(end)); 
    }


    Degrees Video::end_angle() const                   
    { 
        return to_angle(to_uint16_host(end_azimuth)); 
    }

    
    void Video::start_bin(Unit::Bin first)
    {
        range_start = to_uint32_network(static_cast<std::uint32_t>(first));
    }


    Unit::Bin Video::start_bin() const
    {
        return to_uint32_host(range_start);
    } 


    std::size_t Video::num_bins() const
    {
        return video_size_cells.to_uint32();
    }


    void Video::num_bins(std::size_t bins)
    {
        video_size_cells = static_cast<std::uint32_t>(bins);
    }


    Unit::Metre Video::bin_size() const                
    { 
        return to_metre(to_uint32_host(cell_duration)); 
    }


    void Video::bin_size(Unit::Metre m)                
    { 
        cell_duration = to_uint32_network(to_fsec(m)); 
    }


    inline std::vector<std::uint8_t>& operator<<(std::vector<std::uint8_t>& v, const Video& fft)
    {
        v.insert(v.end(), fft.begin(), fft.end());
        return v;
    }


    std::size_t Video::video_size() const
    {
        return to_uint16_host(video_size_bytes);
    }


    void Video::video_size(std::size_t sz)
    {
        video_size_bytes = to_uint16_network(static_cast<std::uint16_t>(sz));
    }

    
    std::size_t Video::update_video_info(std::size_t data_sz)
    {
        // The *actual* payload size must be a number of fixed-size blocks, based on 
        // the size of the incoming data buffer.
        // The block size will be 4, 64 or 256 bytes.  If the data doesn't fit into a 
        // whole number of blocks, the final block must be zero-padded.
        // This function returns the *actual* size, as num_blocks * block_size

        video_size(data_sz);
        update_block_info(data_sz);

        num_blocks = (data_sz / block_size()) + ((data_sz % block_size() != 0) ? 1 : 0);

        // We only use 8/16-bit resolution
        //
        // auto cell_sz = uint24_t::overlay_at(reinterpret_cast<std::uint8_t*>(&video_size_cells));
        video_size_cells = (data_sz / (bit_resolution() / 8));

        return video_block_size();
    }


    std::vector<std::uint8_t> Video::video_to_vector() const
    {
        // FFT data starts immediately after the
        // Video header.
        //
        auto fft_start = end();
        auto fft_end   = fft_start + video_size();
        
        return { fft_start, fft_end }; 
    }


    std::size_t Video::block_size() const
    {
        auto flags = features();
        if (flags.video_low_volume == 1)      return 4;
        if (flags.video_medium_volume == 1)   return 64;
        if (flags.video_high_volume == 1)     return 256;
        return { };
    }


    std::size_t Video::video_block_size() const
    {
        return block_count() * block_size();
    }
    
    
    void Video::update_block_info(std::size_t data_sz)
    {
        auto flags = features();
        if (data_sz <= 1020)                    flags.video_low_volume    = 1;
        if (data_sz > 1020 && data_sz <= 16320) flags.video_medium_volume = 1;
        if (data_sz > 16320)                    flags.video_high_volume   = 1;
        features(flags);
    }


    // Helpers
    //
    int Video::bit_resolution() const
    {
        return (1 << (resolution() - 1));
    }

    constexpr std::uint32_t Video::to_fsec(Unit::Metre s) const
    {
        constexpr Unit::Metres_per_sec c    { 299'792'458.0 };
        constexpr float                fsec { 1e15 };

        return ((s / (c / 2)) * fsec); 
    }

    constexpr Unit::Metre Video::to_metre(std::uint32_t dur_fsec) const
    {
        constexpr Unit::Metres_per_sec c    { 299'792'458.0 };
        constexpr float                fsec { 1e15 };

        return (dur_fsec / fsec) * (c / 2); 
    }


    constexpr std::uint16_t Video::angle_to_uint16(const Unit::Degrees& angle) const
    {
        return static_cast<std::uint16_t>(angle.to_float() * ((1 << 16) / 360.0));
    }


    constexpr Unit::Degrees Video::to_angle(std::uint16_t value) const
    {
        return Unit::Degrees { static_cast<float>(value) * (360.0f / (1 << 16)) };
    }


// DO NOT REMOVE - 
// This ensures correct alignment for all messages
//
#pragma pack()

} // namespace Navtech::Networking::Colossus_protocol


#endif // CAT240_VIDEO_MESSAGE_H
