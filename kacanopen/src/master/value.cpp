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
#include "kacanopen/core/canopen_error.h"
 
#include "kacanopen/master/value.h"

#include <sstream>
#include <string>

namespace kaco {

Value::Value() {
	DEBUG_LOG("Creating empty value");
	type = Type::invalid;
}

Value::Value(uint8_t value) {
	DEBUG_LOG("Creating uint8 value");
	type = Type::uint8;
	uint8 = value;
}

Value::Value(uint16_t value) {
	DEBUG_LOG("Creating uint16 value");
	type = Type::uint16;
	uint16 = value;
}

Value::Value(uint32_t value) {
	DEBUG_LOG("Creating uint32 value");
	type = Type::uint32;
	uint32 = value;
}

Value::Value(uint64_t value) {
	DEBUG_LOG("Creating uint64 value");
	type = Type::uint64;
	uint64 = value;
}

Value::Value(int8_t value) {
	DEBUG_LOG("Creating int8 value");
	type = Type::int8;
	int8 = value;
}

Value::Value(int16_t value) {
	DEBUG_LOG("Creating int16 value");
	type = Type::int16;
	int16 = value;
}

Value::Value(int32_t value) {
	DEBUG_LOG("Creating int32 value");
	type = Type::int32;
	int32 = value;
}

Value::Value(int64_t value) {
	DEBUG_LOG("Creating int64 value");
	type = Type::int64;
	int64 = value;
}

Value::Value(float value) {
	DEBUG_LOG("Creating real32 value");
	type = Type::real32;
	real32 = value;
}

Value::Value(double value) {
	DEBUG_LOG("Creating real64 value");
	type = Type::real64;
	real64 = value;
}

Value::Value(bool value) {
	DEBUG_LOG("Creating boolean value");
	type = Type::boolean;
	boolean = value;
}

Value::Value(const std::string& value) : string(value) {
	DEBUG_LOG("Creating string value");
	type = Type::string;
}

Value::Value(const char* value) : string(value) {
	DEBUG_LOG("Creating string value");
	type = Type::string;
}

Value::Value(const std::vector<uint8_t>& value) : octet_string(value) {
	DEBUG_LOG("Creating octet string value");
	type = Type::octet_string;
}

Value::Value(Type type_, const std::vector<uint8_t>& data) {

	type = type_;
	
	if (type != Type::string && type != Type::octet_string) {
		// strings and octet strings have variable size
		const uint8_t type_size = Utils::get_type_size(type);
		if (data.size() != type_size) {
			throw canopen_error("[Value constructor] Wrong byte vector size: data.size()="+std::to_string(data.size())
				+" type_size="+std::to_string(type_size)+" type="+Utils::type_to_string(type));
		}
	}

	switch(type) {
			
		case Type::uint8: {
			uint8 = data[0];
			break;
		}
			
		case Type::int8: {
			int8 = data[0];
			break;
		}

		case Type::uint16: {
			uint16 = (uint16_t)data[0] + ((uint16_t)data[1] << 8);
			break;
		}

		case Type::int16: {
			int16 = (int16_t)data[0] + ((int16_t)data[1] << 8);
			break;
		}

		case Type::uint32: {
			uint32 = (uint32_t)data[0] + ((uint32_t)data[1] << 8) + ((uint32_t)data[2] << 16) + ((uint32_t)data[3] << 24);
			break;
		}

		case Type::int32: {
			int32 = (int32_t)data[0] + ((int32_t)data[1] << 8) + ((int32_t)data[2] << 16) + ((int32_t)data[3] << 24);
			break;
		}

		case Type::uint64: {
			uint64 = (uint64_t)data[0] + ((uint64_t)data[1] << 8) + ((uint64_t)data[2] << 16) + ((uint64_t)data[3] << 24)
				+ ((uint64_t)data[4] << 32) + ((uint64_t)data[5] << 40) + ((uint64_t)data[6] << 48) + ((uint64_t)data[7] << 56);
			break;
		}

		case Type::int64: {
			int64 = (int64_t)data[0] + ((int64_t)data[1] << 8) + ((int64_t)data[2] << 16) + ((int64_t)data[3] << 24)
				+ ((int64_t)data[4] << 32) + ((int64_t)data[5] << 40) + ((int64_t)data[6] << 48) + ((int64_t)data[7] << 56);
			break;
		}

		case Type::real32: {
			// TODO: test this
			const uint32_t val = (uint32_t)data[0] + ((uint32_t)data[1] << 8) + ((uint32_t)data[2] << 16) + ((uint32_t)data[3] << 24);
			real32 = reinterpret_cast<const float&>(val);
			break;	
		}

		case Type::real64: {
			// TODO: test this
			const uint64_t val = (uint64_t)data[0] + ((uint64_t)data[1] << 8) + ((uint64_t)data[2] << 16) + ((uint64_t)data[3] << 24)
				+ ((uint64_t)data[4] << 32) + ((uint64_t)data[5] << 40) + ((uint64_t)data[6] << 48) + ((uint64_t)data[7] << 56);
			real64 = reinterpret_cast<const double&>(val);
			break;
		}

		case Type::string: {
			string = std::string(reinterpret_cast<char const*>(data.data()), data.size());
			break;
		}

		case Type::octet_string: {
			octet_string = data;
			break;
		}

		case Type::boolean: {
			boolean = (data[0]>0);
			break;
		}

		case Type::invalid: {
			throw canopen_error("[Value constructor] Can't construct invalid value with this constructor.");
		}

	}

}

std::vector<uint8_t> Value::get_bytes() const {

	std::vector<uint8_t> result;

	switch(type) {

		case Type::uint8: {
			result.push_back(uint8);
			break;
		}

		case Type::uint16: {
			result.push_back(uint16 & 0xFF);
			result.push_back((uint16>>8) & 0xFF);
			break;
		}

		case Type::uint32: {
			result.push_back(uint32 & 0xFF);
			result.push_back((uint32>>8) & 0xFF);
			result.push_back((uint32>>16) & 0xFF);
			result.push_back((uint32>>24) & 0xFF);
			break;
		}

		case Type::uint64: {
			result.push_back(uint64 & 0xFF);
			result.push_back((uint64>>8) & 0xFF);
			result.push_back((uint64>>16) & 0xFF);
			result.push_back((uint64>>24) & 0xFF);
			result.push_back((uint64>>32) & 0xFF);
			result.push_back((uint64>>40) & 0xFF);
			result.push_back((uint64>>48) & 0xFF);
			result.push_back((uint64>>56) & 0xFF);
			break;
		}

		case Type::int8: {
			result.push_back(int8);
			break;
		}

		case Type::int16: {
			result.push_back(int16 & 0xFF);
			result.push_back((int16>>8) & 0xFF);
			break;
		}

		case Type::int32: {
			result.push_back(int32 & 0xFF);
			result.push_back((int32>>8) & 0xFF);
			result.push_back((int32>>16) & 0xFF);
			result.push_back((int32>>24) & 0xFF);
			break;
		}

		case Type::int64: {
			result.push_back(int64 & 0xFF);
			result.push_back((int64>>8) & 0xFF);
			result.push_back((int64>>16) & 0xFF);
			result.push_back((int64>>24) & 0xFF);
			result.push_back((int64>>32) & 0xFF);
			result.push_back((int64>>40) & 0xFF);
			result.push_back((int64>>48) & 0xFF);
			result.push_back((int64>>56) & 0xFF);
			break;
		}

		case Type::real32: {
			// TODO: test this
			const uint32_t val = reinterpret_cast<const uint32_t&>(real32);
			result.push_back(val & 0xFF);
			result.push_back((val>>8) & 0xFF);
			result.push_back((val>>16) & 0xFF);
			result.push_back((val>>24) & 0xFF);
			break;
		}

		case Type::real64: {
			// TODO: test this
			const uint64_t val = reinterpret_cast<const uint64_t&>(real64);
			result.push_back(val & 0xFF);
			result.push_back((val>>8) & 0xFF);
			result.push_back((val>>16) & 0xFF);
			result.push_back((val>>24) & 0xFF);
			result.push_back((val>>32) & 0xFF);
			result.push_back((val>>40) & 0xFF);
			result.push_back((val>>48) & 0xFF);
			result.push_back((val>>56) & 0xFF);
			break;
		}

		case Type::boolean: {
			result.push_back(boolean?0x01:0x00);
			break;
		}

		case Type::string: {
			for (size_t i=0; i<string.length(); ++i) {
				result.push_back((uint8_t)string[i]);
			}
			break;
		}

		case Type::octet_string: {
			result = octet_string;
			break;
		}

		case Type::invalid: {
			throw canopen_error("[Value::get_bytes] Can't get bytes of invalid type.");
		}

	}

	return result;

}

bool Value::operator==(const Value& other) const {

	DEBUG(

		if (type != other.type) {
			throw canopen_error("[Value::operator==] Comparing values of different type: "+Utils::type_to_string(type)+" != "+Utils::type_to_string(other.type)+".");
		}

	)

	switch(type) {

		case Type::uint8: {
			return uint8 == (uint8_t) other;
		}

		case Type::uint16: {
			return uint16 == (uint16_t) other;
		}

		case Type::uint32: {
			return uint32 == (uint32_t) other;
		}

		case Type::uint64: {
			return uint64 == (uint64_t) other;
		}
			
		case Type::int8: {
			return int8 == (int8_t) other;
		}

		case Type::int16: {
			return int16 == (int16_t) other;
		}

		case Type::int32: {
			return int32 == (int32_t) other;
		}

		case Type::int64: {
			return int64 == (int64_t) other;
		}

		case Type::real32: {
			return real32 == (float) real32;
		}

		case Type::real64: {
			return real64 == (double) real64;
		}

		case Type::boolean: {
			return boolean == (bool) boolean;
		}

		case Type::string: {
			return string == (std::string) other;
		}

		case Type::octet_string: {
			// reusing code from value_printer...
			return to_string() == other.to_string();
		}

		case Type::invalid: {
			throw canopen_error("[Value::operator==] Comparing invalid type.");
		}

	}
	return false;
}

bool Value::operator!=(const Value& other) const {
	return !(operator==(other));
}

std::string Value::to_string() const {
	using namespace value_printer;
	std::ostringstream stream;
	stream << std::dec << *this;
	return stream.str();
}

//----------------//
// Cast operators //
//----------------//

/// converts a CanOpen type <mtypein> and a C++ type <mtypeout>
/// to a cast operator
#define CO_VALUE_TYPE_CAST_OP(mtypeout, mtypein) \
	Value::operator mtypeout() const { \
		if (type != Type::mtypein ) { \
			throw canopen_error("[Value cast operator] Illegal conversion from "+Utils::type_to_string(type)+" to " #mtypein "."); \
		} \
		return mtypein; \
	}

/// converts a CanOpen type <mtypein> to a cast method to
/// the C++ output type <mtypein>_t
#define CO_VALUE_TYPE_CAST_OP_INT(mtype) CO_VALUE_TYPE_CAST_OP(mtype##_t, mtype)

CO_VALUE_TYPE_CAST_OP_INT(uint8)
CO_VALUE_TYPE_CAST_OP_INT(uint16)
CO_VALUE_TYPE_CAST_OP_INT(uint32)
CO_VALUE_TYPE_CAST_OP_INT(uint64)
CO_VALUE_TYPE_CAST_OP_INT(int8)
CO_VALUE_TYPE_CAST_OP_INT(int16)
CO_VALUE_TYPE_CAST_OP_INT(int32)
CO_VALUE_TYPE_CAST_OP_INT(int64)
CO_VALUE_TYPE_CAST_OP(bool, boolean)
CO_VALUE_TYPE_CAST_OP(float, real32)
CO_VALUE_TYPE_CAST_OP(double, real64)
CO_VALUE_TYPE_CAST_OP(std::string, string)
CO_VALUE_TYPE_CAST_OP(std::vector<uint8_t>, octet_string)

//-------------------//
// std::cout Printer //
//-------------------//

namespace value_printer {
	std::ostream &operator<<(std::ostream &os, Value val) {
		switch(val.type) {
				
			case Type::uint8: {
				// 1-byte types are printed as char by default -> double casting.
				return os << static_cast<uint32_t>(static_cast<uint8_t>(val));
			}

			case Type::uint16: {
				return os << static_cast<uint16_t>(val);
			}

			case Type::uint32: {
				return os << static_cast<uint32_t>(val);
			}

			case Type::uint64: {
				return os << static_cast<uint64_t>(val);
			}
				
			case Type::int8: {
				// 1-byte types are printed as char by default -> double casting.
				return os << static_cast<uint32_t>(static_cast<int8_t>(val));
			}

			case Type::int16: {
				return os << static_cast<int16_t>(val);
			}

			case Type::int32: {
				return os << static_cast<int32_t>(val);
			}

			case Type::int64: {
				return os << static_cast<int64_t>(val);
			}

			case Type::real32: {
				return os << static_cast<float>(val);
			}

			case Type::real64: {
				return os << static_cast<double>(val);
			}

			case Type::boolean: {
				return os << (static_cast<bool>(val)?"TRUE":"FALSE");
			}

			case Type::string: {
				return os << static_cast<std::string>(val);
			}

			case Type::octet_string: {
				const auto& octet_string = val.get_bytes();
				os << "[";
				for (size_t i=0; i<octet_string.size(); ++i) {
					// 1-byte types are printed as char by default -> casting.
					os << static_cast<uint32_t>(octet_string[i]);
					if (i+1<octet_string.size()) {
						os << ",";
					}
				}
				return os << "]";
			}

			default: {
				return os << "[Unknown value type]";
			}

		}
	}
}

} // end namespace kaco