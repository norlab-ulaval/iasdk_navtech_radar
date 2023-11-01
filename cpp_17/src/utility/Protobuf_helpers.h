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

#ifndef PROTOBUF_HELPERS_H
#define PROTOBUF_HELPERS_H

#include <google/protobuf/text_format.h>

#include <string>
#include <memory>
#include <cstdint>
#include <vector>
#include <optional>


namespace Navtech::Protobuf {

	enum class Format { text, binary };

	// Generate a string containing a (human readable) serialized 
	// protocol buffer.
	//
	template <typename Protobuf_Ty>
	std::optional<std::string> string_from(const Protobuf_Ty& proto_buf)
	{
		try {
			std::string output_str { };

			if (google::protobuf::TextFormat::PrintToString(proto_buf, &output_str)) {
				return output_str;
			}
			else {
				return std::nullopt;
			}
		}
		catch (std::exception&) {
			return std::nullopt;
		}
	}


	// Serialise an object to its protocol buffer form, then generate
	// a (human readable) text string from that protocol buffer.  This
	// function allows an object to be serialize to a file (for example)
	// Note: the object_to_serialize must support the to_protocol_buffer
	// method.
	//
	template <typename Protobuf_Ty, typename T>
	std::optional<std::string> to_string_as(T& object_to_serialize)
	{
		std::string result { };
		try {
			Protobuf_Ty proto_buf { };
			object_to_serialize.to_protocol_buffer(&proto_buf);
			return string_from<Protobuf_Ty>(proto_buf);
		}
		catch (std::exception&) {
			return std::nullopt;
		}
	}


	// De-serialise a text string back into a protocol buffer.
	//
	template <class Protobuf_Ty>
	std::optional<Protobuf_Ty> from_string_into(const std::string& input_str)
	{
		try {
			Protobuf_Ty proto_buf { };

			if (google::protobuf::TextFormat::ParseFromString(input_str, &proto_buf)) {
				return proto_buf;
			}
			else {
				return std::nullopt;
			}
		}
		catch (std::exception&) {
			return std::nullopt;
		}
	}


	// Generate a vector of bytes containing a serialized 
	// protocol buffer.
	// The client has the option to serialise the protocol buffer in 
	// either text (human readable) or binary (the default for network
	// transmission) format.
	//
	template <class Protobuf_Ty>
	std::optional<std::vector<uint8_t>> vector_from(Protobuf_Ty& proto_buf, Format format = Format::binary)
	{
		try {
			std::string output_str { };
			bool printed { false };

			if (format == Format::binary) {
				printed = proto_buf.SerializeToString(&output_str);
			}
			else {
				printed = google::protobuf::TextFormat::PrintToString(proto_buf, &output_str);
			}

			if (printed) return std::vector<std::uint8_t> { output_str.begin(), output_str.end() };
			else 		 return std::nullopt;
		}
		catch (std::exception&) {
			return std::nullopt;
		}
	}


	// Serialise an object to its protocol buffer form, then generate
	// a vector from that protocol buffer. This function allows an object 
	// to be serialized for network transmission (for example).
	// Note: the object_to_serialize must support the to_protocol_buffer
	// method.
	//
	template <typename Protobuf_Ty, typename T>
	std::optional<std::vector<uint8_t>> to_vector_as(T& object_to_serialize, Format format = Format::binary)
	{
		std::string result { };
		try {
			Protobuf_Ty proto_buf { };
			object_to_serialize.to_protocol_buffer(&proto_buf);
			return vector_from<Protobuf_Ty>(proto_buf, format);
		}
		catch (std::exception&) {
			return std::nullopt;
		}
	}



	// De-serialise a vector of bytes back into a protocol buffer.
	//
	template <class Protobuf_Ty>
	std::optional<Protobuf_Ty> from_vector_into(const std::vector<uint8_t>& data, Format format = Format::binary)
	{
		try {
			Protobuf_Ty proto_buf { };
			bool parsed { false };

			std::string input_str { data.begin(), data.end() };

			if (format == Format::binary) {
				parsed = proto_buf.ParseFromString(input_str);
			}
			else {	
				parsed = google::protobuf::TextFormat::ParseFromString(input_str, &proto_buf);
			}

			if (parsed) return proto_buf;
			else        return std::nullopt;
		}
		catch (std::exception&) {
			return std::nullopt;
		}
	}

} // namespace Navtech::Protobuf

#endif // PROTOBUF_HELPERS_H

