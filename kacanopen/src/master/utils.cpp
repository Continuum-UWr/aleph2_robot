/*
 * Copyright (c) 2015, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "kacanopen/core/logger.h"

#include "kacanopen/master/utils.h"

#include <string>
#include <cstdint>
#include <algorithm>
#include <stdexcept>

namespace kaco {

std::string Utils::type_to_string(Type type) {
	switch(type) {
		case Type::uint8:
			return "uint8";
		case Type::uint16:
			return "uint16";
		case Type::uint32:
			return "uint32";
		case Type::uint64:
			return "uint64";
		case Type::int8:
			return "int8";
		case Type::int16:
			return "int16";
		case Type::int32:
			return "int32";
		case Type::int64:
			return "int64";
		case Type::real32:
			return "real32";
		case Type::real64:
			return "real64";
		case Type::boolean:
			return "boolean";
		case Type::string:
			return "string";
		case Type::octet_string:
			return "octet_string";
		default:
			return "unknown type";
	}
}

std::string Utils::data_type_to_string(DataType type) {
	#define ENUM_CASE(name) case DataType::name: return #name;
	switch(type) {
		ENUM_CASE(BOOLEAN)
		ENUM_CASE(INTEGER8)
		ENUM_CASE(INTEGER16)
		ENUM_CASE(INTEGER32)
		ENUM_CASE(UNSIGNED8)
		ENUM_CASE(UNSIGNED16)
		ENUM_CASE(UNSIGNED32)
		ENUM_CASE(REAL32)
		ENUM_CASE(VISIBLE_STRING)
		ENUM_CASE(OCTET_STRING)
		ENUM_CASE(UNICODE_STRING)
		ENUM_CASE(TIME_OF_DAY)
		ENUM_CASE(TIME_DIFFERENCE)
		ENUM_CASE(LARGEDATA)
		ENUM_CASE(INTEGER24)
		ENUM_CASE(REAL64)
		ENUM_CASE(INTEGER40)
		ENUM_CASE(INTEGER48)
		ENUM_CASE(INTEGER56)
		ENUM_CASE(INTEGER64)
		ENUM_CASE(UNSIGNED40)
		ENUM_CASE(UNSIGNED48)
		ENUM_CASE(UNSIGNED56)
		ENUM_CASE(UNSIGNED64)
		ENUM_CASE(PDO_COMMUNICATION_PARAMETER)
		ENUM_CASE(PDO_MAPPING)
		ENUM_CASE(SDO_PARAMETER)
		ENUM_CASE(IDENTITY)
		default:
			return "[Utils::data_type_to_string] Unknown type with id "+std::to_string(static_cast<uint16_t>(type));
	}
	#undef ENUM_CASE
}

uint8_t Utils::get_type_size(Type type) {
	switch(type) {
		case Type::uint8:
		case Type::int8:
		case Type::boolean:
			return 1;
		case Type::uint16:
		case Type::int16:
			return 2;
		case Type::uint32:
		case Type::int32:
		case Type::real32:
			return 4;
		case Type::uint64:
		case Type::int64:
		case Type::real64:
			return 8;
		case Type::string:
		case Type::octet_string:
		default:
			ERROR("[Utils::get_type_size] Unknown type or type with variable size.");
			return 0;
	}
}

Type Utils::type_code_to_type(uint16_t code) {
	switch (code) {
		case (uint16_t) DataType::BOOLEAN: return Type::boolean;
		case (uint16_t) DataType::INTEGER8: return Type::int8;
		case (uint16_t) DataType::INTEGER16: return Type::int16;
		case (uint16_t) DataType::INTEGER32: return Type::int32;
		case (uint16_t) DataType::UNSIGNED8: return Type::uint8;
		case (uint16_t) DataType::UNSIGNED16: return Type::uint16;
		case (uint16_t) DataType::UNSIGNED32: return Type::uint32;
		case (uint16_t) DataType::REAL32: return Type::real32;
		case (uint16_t) DataType::VISIBLE_STRING: return Type::string;
		case (uint16_t) DataType::OCTET_STRING: return Type::octet_string;
		//case UNICODE_STRING: return Type::;
		//case TIME_OF_DAY: return Type::;
		//case TIME_DIFFERENCE: return Type::;
		//case LARGEDATA: return Type::;
		//case INTEGER24: return Type::;
		case (uint16_t) DataType::REAL64: return Type::real64;
		//case INTEGER40: return Type::;
		//...
		case (uint16_t) DataType::INTEGER64: return Type::int64;
		case (uint16_t) DataType::UNSIGNED64: return Type::uint64;
		//...
		default:
			ERROR("[Utils::type_code_to_type] Data type "<<data_type_to_string(static_cast<DataType>(code))<<" not yet supported.");
			return Type::invalid;
	}
}

std::string Utils::escape(const std::string& str) {
	std::string out = str;
	std::transform(out.begin(), out.end(), out.begin(), ::tolower);
	std::replace(out.begin(), out.end(), ' ', '_');
	std::replace(out.begin(), out.end(), '-', '_');
	return out;
}

unsigned long long Utils::hexstr_to_uint(std::string str) {
	try {
		return std::stoull(str, nullptr, 16);
	} catch ( const std::invalid_argument& e ) {
		ERROR("[Utils::hexstr_to_uint] Invalid argument: \""<<str<<"\" ("<<e.what()<<")");
		return 0;
	} catch ( const std::out_of_range& e ) {
		ERROR("[Utils::hexstr_to_uint] Out of range: \""<<str<<"\" ("<<e.what()<<")");
		return 0;
	}
}

unsigned long long Utils::decstr_to_uint(std::string str) {
	try {
		return std::stoull(str, nullptr, 10);
	} catch ( const std::invalid_argument& e ) {
		ERROR("[Utils::decstr_to_uint] Invalid argument: \""<<str<<"\" ("<<e.what()<<")");
		return 0;
	} catch ( const std::out_of_range& e ) {
		ERROR("[Utils::decstr_to_uint] Out of range: \""<<str<<"\" ("<<e.what()<<")");
		return 0;
	}
}

AccessType Utils::string_to_access_type(std::string str) {
	if (str == "ro") {
		return AccessType::read_only;
	} else if (str == "wo") {
		return AccessType::write_only;
	} else if (str == "const") {
		return AccessType::constant;
	} else if (str == "rw" || str == "rwr" || str == "rww") {
		return AccessType::read_write;
	} else {
		ERROR("[Utils::str_to_access_type] Invalid access type string \""<<str<<"\". Returning AccessType::read_write.")
		return AccessType::read_write;
	}
}

std::string Utils::access_type_to_string(AccessType type) {
	switch(type) {
		case AccessType::read_only: return "ro";
		case AccessType::write_only: return "wo";
		case AccessType::constant: return "const";
		case AccessType::read_write: return "rw";
		default:
			ERROR("[Utils::access_type_to_string] Unknown access type!");
			return "unknown access type";
	}
}

} // end namespace kaco