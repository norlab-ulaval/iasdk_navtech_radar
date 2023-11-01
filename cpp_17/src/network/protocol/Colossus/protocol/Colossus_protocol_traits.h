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

#ifndef COLOSSUS_PROTOCOL_TRAITS_H
#define COLOSSUS_PROTOCOL_TRAITS_H

#include "Colossus_network_message.h"
#include "Protocol_traits.h"

namespace Navtech::Networking {

	template <>
	class Protocol_traits<Navtech::Networking::Protocol::colossus> {
	public:
		using Message           = Navtech::Networking::Colossus_protocol::Message;
        using Pointer           = Navtech::Networking::Colossus_protocol::Pointer;
		using ID                = Message::ID;
		using Type              = Navtech::Networking::Colossus_protocol::Type;
        using Buffer            = std::vector<std::uint8_t>;
        using Iterator          = std::uint8_t*;
        using Const_iterator    = const std::uint8_t*;

        // ---------------------------------------------------
		
		static constexpr const char* name { "Colossus" };

		// ---------------------------------------------------

        template <typename... Arg_Ty>
        static Message make(Arg_Ty&&... args)
        {
            return Message { std::forward<Arg_Ty>(args)... };
        }

        static void clear(Message& msg)
        {
            msg.relinquish();
        }

		static void replace_data(Message& inout_msg, const Buffer& in_buffer)
        {
            inout_msg.replace(in_buffer);
        }

		static void replace_data(Message& inout_msg, Buffer&& in_buffer)
        {
            inout_msg.replace(std::move(in_buffer));
        }

        static void add_header(Message& inout_msg, const Buffer& in_buffer)
		{
			inout_msg.replace(in_buffer);
        }

        static void add_header(Message& inout_msg, Buffer&& in_buffer)
		{
			inout_msg.replace(std::move(in_buffer));
        }

        static void add_payload(Message& inout_msg, const Buffer& in_buffer)
		{
		    inout_msg.append(in_buffer);
		}

        static void add_payload(Message& inout_msg, Buffer&& in_buffer)
		{
		    inout_msg.append(std::move(in_buffer));
		}

		static void add_ip_address(Message& inout_msg, const Networking::IP_address& in_addr)
		{
			inout_msg.ip_address(in_addr);
		}

		static ID client_id(const Message& in_msg)
		{
			return in_msg.client_id();
		}

		static ID client_id(const Pointer& in_msg)
		{
			return in_msg->client_id();
		}

        static void add_client_id(Message& inout_msg, int in_id)
		{
			inout_msg.client_id(in_id);
		}

		static void add_client_id(Pointer& inout_msg, int in_id)
		{
			inout_msg->client_id(in_id);
		}

		static Type type(const Message& in_msg)
		{
			return in_msg.type();
		}

		static Type type(const Pointer& in_msg)
		{
			return in_msg->type();
		}
        
        static std::size_t header_size (const Message& in_msg [[maybe_unused]])
		{
			return in_msg.header_size();
		}

        static std::size_t payload_size(const Message& in_msg)
		{
			return in_msg.payload_size();
		}
        
        static bool is_valid(const Message& in_msg)
		{
			return in_msg.is_valid();
		}

		static bool is_complete(const Message& in_msg)
		{
			return (in_msg.size() == (in_msg.payload_size() + in_msg.header_size()));
		}

        static Pointer dyn_alloc()
		{
			return allocate_shared<Message>();
		}

        static Buffer to_buffer(const Message& in_msg)
        {
            return in_msg.data_view();
        }

		static Buffer to_buffer(Message&& in_msg)
        {
            return in_msg.relinquish();
        }
	};

} // namespace Navtech::Networking

#endif // COLOSSUS_PROTOCOL_TRAITS_H