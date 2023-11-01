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

#ifndef COLOSSUS_MESSAGE_BASE_H
#define COLOSSUS_MESSAGE_BASE_H

#include <cstdint>
#include <cstddef>

#include "Colossus_network_message.h"
#include "pointer_types.h"

namespace Navtech::Networking::Colossus_protocol {

    // -------------------------------------------------------------------------------------------
    // Base types for messages.  This implementation uses the Curiously Recurring Template Pattern
    // (CRTP).  This allows a set of common base types, with core functionality, and yet still
    // retain unique message interfaces, without requiring (dynamic) downcasting from the client
    // code.
    //
    namespace Message_base {

        template <typename Derived_Ty>
        class Header_and_payload;

        #pragma pack(1)
        template <typename Derived_Ty>
        class Header_only {
        public:
    
            // Derived types MUST override this function to return the size
            // of their header (in bytes)
            //
            std::size_t header_size() const
            {
                return actual().size();
            }

            // begin() and end() give pointers to the Derived_Ty
            // object.
            //
            const std::uint8_t* begin() const
            {
                return (
                    reinterpret_cast<const std::uint8_t*>(this) + 
                    sizeof(Header_only)
                );
            }

            std::uint8_t* begin()
            {
                return (
                    reinterpret_cast<std::uint8_t*>(this) + 
                    sizeof(Header_only)
                );
            }

            const std::uint8_t* end() const
            {
                return begin() + header_size();
            }

            std::uint8_t* end()
            {
                return begin() + header_size();
            }

        protected:
            friend Header_and_payload<Derived_Ty>;

            Derived_Ty& actual()
            {
                return *(reinterpret_cast<Derived_Ty*>(this));
            }

            const Derived_Ty& actual() const
            {
                return *(reinterpret_cast<const Derived_Ty*>(this));
            }

            const std::uint8_t* payload_begin() const
            {
                return end();
            }

            const std::uint8_t* payload_end() const
            {
                return end() + payload_size() - header_size();
            }

            std::size_t payload_size() const
            {
                return ntohl(header.payload_sz);
            }

            std::size_t size() const
            {
                return 0;
            }

            // -----------------------------------------------------------------------------------
            // This header overlays onto the actual Colossus network message header
            // and gives access to the payload size, for computing payload iterators
            //
            struct Header {
                static constexpr auto padding_sz = Message::header_size() - sizeof(std::uint32_t);
                std::byte     alignment_padding[padding_sz];
                std::uint32_t payload_sz;
            };
            
            const Header header { };
        };
        #pragma pack()

        #pragma pack(1)
        template <typename Derived_Ty>
        class Header_and_payload : public Header_only<Derived_Ty> {
        public:
            std::string to_string() const
            {
                return std::string { protobuf_begin(), protobuf_end() };
            }

            std::vector<std::uint8_t> to_vector() const
            {
                return std::vector<std::uint8_t> { protobuf_begin(), protobuf_end() };
            }

            std::size_t protobuf_size() const
            {
               return protobuf_end() - protobuf_begin();
            }

        protected:
            using Header = Header_only<Derived_Ty>;

        
            const std::uint8_t* protobuf_begin() const
            {
                return Header::end();
            }
            
            const std::uint8_t* protobuf_end() const
            {
                return Header::payload_end();
            }
        };
        #pragma pack()


        template <typename Derived_Ty>
        using Payload_only = Header_and_payload<Derived_Ty>;

        template <typename Derived_Ty>
        using Simple = Payload_only<Derived_Ty>;

    } // namespace Message_base
    
} //namespace Navtech::Networking::Colossus_protocol


#endif // COLOSSUS_MESSAGE_BASE_H