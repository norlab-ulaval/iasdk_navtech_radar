#include "Cat240_server.h"
#include "Message_buffer.h"

#include "Memory_types.h"



namespace Navtech::Networking::Cat240_protocol {

    Server::Server() :
        Active          { "CAT-240 server" },
        datagram_server { local_endpt, event_dispatcher }
    {
    }


    Server::Server(const Endpoint& remote_endpt) :
        Active          { "CAT-240 server" },
        datagram_server { local_endpt, remote_endpt, event_dispatcher }
    {
    }


    Server::Server(const Endpoint& remote_endpt, std::size_t max_packet_size) :
        Active          { "CAT-240 server" },
        max_packet_sz   { max_packet_size },
        datagram_server { local_endpt, remote_endpt, event_dispatcher }
    {
    }


    void Server::on_start()
    {
        msg_handler.when_notified_invoke(
            [this](const Message& msg) 
            { 
                async_call(&Server::on_received_msg, this, msg); 
            }
        );
        event_dispatcher.attach_to<Event::received_message>(msg_handler);

        datagram_server.start();
    }


    void Server::on_stop()
    {
        event_dispatcher.detach_from<Event::received_message>(msg_handler);

        datagram_server.stop();
        datagram_server.join();
    }


    std::size_t Server::max_packet_size() const
    {
        return max_packet_sz;
    }
    
    
    void Server::max_packet_size(std::size_t sz)
    {
        max_packet_sz = sz;
    }


    void Server::send_to(const Endpoint& remote)
    {
        datagram_server.send_to(remote);
    }


    void Server::send(const Cat240_protocol::Message& msg)
    {
        async_call(&Server::on_send, this, msg);
    }


    void Server::send(Cat240_protocol::Message&& msg)
    {
        async_call(&Server::on_send, this, std::move(msg));
    }


    void Server::on_send(Cat240_protocol::Message& msg)
    {
        // If the message is bigger than the MTU, split it into
        // N smaller messages.
        //
        if (msg.size() > max_packet_sz) send_as_multiple(msg);
        else                            send_as_single(msg);
    }


    void Server::send_as_single(Cat240_protocol::Message& msg)
    {
        using Cat240_protocol::Header;
        using Cat240_protocol::Video;

        auto header = Header::overlay_at(msg.begin());

        if (header->type() == Cat240_protocol::Type::video) {
            auto video = Video::overlay_at(msg.begin());
            video->sweep_counter(next_index());
        }

        datagram_server.send(std::move(msg));
    }


    void Server::send_as_multiple(Cat240_protocol::Message& msg)
    {
        using Navtech::Networking::Message_buffer;
        using Navtech::Networking::Buffer_view;
        using Cat240_protocol::Video;
        using Cat240_protocol::Time_of_day;
        using Cat240_protocol::Extended_info;
        using Cat240_protocol::Message;

        // Extract the common components from message
        //
        Message_buffer header_buffer        { };
        Message_buffer ToD_buffer           { };
        Message_buffer extended_info_buffer { };

        auto itr        = msg.begin();
        auto header     = Video::overlay_at(itr);
        auto features   = header->features();
        header_buffer   = header->to_vector();

        itr = header->video_end();
        auto time_of_day = Time_of_day::overlay_at(itr);
        if (features.time_of_day) ToD_buffer = time_of_day->to_vector();

        itr = time_of_day->end();
        auto extended_info = Extended_info::overlay_at(itr);
        if (features.special_purpose_field) extended_info_buffer = extended_info->to_vector();

        auto payload_buffer = header->video_to_vector();
        auto payload_itr    = payload_buffer.begin();

        auto fixed_fields_sz = non_payload_size(msg);
        auto max_payload_sz  = max_packet_sz - fixed_fields_sz;
        
        auto bytes_remaining = payload_buffer.size();
        Unit::Bin start_bin        { 0 };
        Unit::Bin next_start_bin   { 0 };

        while (bytes_remaining > 0) {
            start_bin = next_start_bin;

            // Determine payload for this particular message
            //
            auto [data_sz, payload_sz, num_blocks ] = payload_split_size(
                bytes_remaining, 
                max_payload_sz, 
                header->block_size()
            );

            Message_buffer payload_n { };
            payload_n.resize(payload_sz);
            copy_n(payload_itr, data_sz, payload_n.begin());

            payload_itr     += data_sz;
            next_start_bin  += data_sz;
            bytes_remaining -= data_sz;

            // Update header
            //
            auto hdr = Video::overlay_at(header_buffer.data());
            hdr->sweep_counter(next_index());
            hdr->message_length(fixed_fields_sz + payload_sz);
            hdr->block_count(num_blocks);
            hdr->start_bin(start_bin);
            hdr->video_size(data_sz);
            if (hdr->resolution() == Video::Resolution::high)      hdr->num_bins(data_sz);
            if (hdr->resolution() == Video::Resolution::very_high) hdr->num_bins(data_sz / 2);


            // Package up the message elements and send.  Note, we are not using
            // the normal messaging API as this (may) modify header internals that
            // we don't want changed in this case.
            //
            Message_buffer buffer { };
            buffer.insert(buffer.end(), header_buffer.begin(), header_buffer.end());
            buffer.insert(buffer.end(), payload_n.begin(), payload_n.end());
            buffer.insert(buffer.end(), ToD_buffer.begin(), ToD_buffer.end());
            buffer.insert(buffer.end(), extended_info_buffer.begin(), extended_info_buffer.end());

            Message msg_n { buffer };
            datagram_server.send(std::move(msg_n));
        }
    }


    std::size_t Server::non_payload_size(const Cat240_protocol::Message& msg)
    {
        using Cat240_protocol::Header;
        using Cat240_protocol::Video;
        using Cat240_protocol::Time_of_day;
        using Cat240_protocol::Extended_info;

        std::size_t fixed_sz { };

        auto header = Header::overlay_at(msg.begin());

        fixed_sz += Video::size();
        if (header->features().time_of_day)           fixed_sz += Time_of_day::size();
        if (header->features().special_purpose_field) fixed_sz += Extended_info::size();

        return fixed_sz;
    }


    std::size_t Server::max_payload_blocks(std::size_t payload_sz, std::size_t block_sz)
    {
        std::size_t count { };
        std::size_t current_size { block_sz };

        while(current_size <= payload_sz) {
            ++count;
            current_size += block_sz;
        }

        return count;
    }


    std::size_t Server::blocks_required(std::size_t payload_sz, std::size_t block_sz)
    {
        auto num_blocks = (payload_sz / block_sz) + ((payload_sz % block_sz != 0) ? 1 : 0);
        return num_blocks;
    }


    std::tuple<
        std::size_t,    // Number of 'valid' data bytes
        std::size_t,    // Payload size, in bytes; always a multiple of the block size
        std::size_t     // Number of blocks
    > 
    Server::payload_split_size(
        std::size_t requested_sz,
        std::size_t max_sz, 
        std::size_t block_sz
    )
    {
        auto num_blocks = blocks_required(requested_sz, block_sz);
        auto max_blocks = max_payload_blocks(max_sz, block_sz);
        auto limited_sz = max_blocks * block_sz;

        if (num_blocks >= max_blocks) return { limited_sz, limited_sz, max_blocks };
        else                          return { requested_sz, (num_blocks * block_sz), num_blocks };
    }


    std::uint32_t Server::next_index()
    {
        // Outgoing (sent) messages may be split and send as multiple
        // messages.  The server must maintian sequential message indexing.
        // Therefore, replace any message's sweep counter index with a new
        // one.  In the case of messages that are not split, the Server's
        // index will be the same at the original message's.
        //
        return index++;
    }


    void Server::on_received_msg(Message& msg)
    {
        using namespace Networking::Cat240_protocol;
		using std::unordered_map;
		using std::begin;
		using std::end;
		using Handler_fn = void (Server::*)(const Message&);

		static const unordered_map<Cat240_protocol::Type, Handler_fn> handlers {
			
            // TODO - 
		};

		auto search_result = handlers.find(msg.type());
		
		if (search_result != end(handlers)) {
			Handler_fn fn = search_result->second;
			(this->*fn)(msg);
		}
		else {
           // TODO - log unhandled message
		}
    }

} // namespace Navtech::Networking::Cat240_protocol