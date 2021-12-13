/*
 * Copyright (c) 2016, Thomas Keh
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
 
#pragma once

#include <cstdint>
#include <functional>


namespace kaco {

	/// Tuple of object dictionary index and subindex
	struct Address {

		/// Index
		uint16_t index;

		/// Sub-index
		uint8_t subindex;

		/// Equality operator for use of Address as key type in std::unordered_map.
		bool operator==(const Address &other) const {
			return (index == other.index && subindex == other.subindex);
		}

	};

} // end namespace kaco

// We put this into the header file because specialization must be parsed before any occurrence of std::unordered_map<Address,...>.
namespace std {

	/// Specialization of std::hash for kaco::Address for use as key type in std::unordered_map.
	template<> struct hash<kaco::Address> {

		/// Argument type
		typedef kaco::Address argument_type;

		/// Result type
		typedef std::size_t result_type;

		/// Hasher
		/// \param s the address to hash
		result_type operator()(argument_type const& s) const {
			const uint32_t a = static_cast<uint32_t>(s.index);
			const uint32_t b = static_cast<uint32_t>(s.subindex);
			return std::hash<uint32_t>()((a<<8)&b);
		}

	};

} // end namespace std