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

#include "kacanopen/core/core.h"
#include "kacanopen/core/logger.h"

#include "kacanopen/master/transmit_pdo_mapping.h"
#include "kacanopen/master/entry.h"
#include "kacanopen/master/dictionary_error.h"

#include <cassert>

namespace kaco {

TransmitPDOMapping::TransmitPDOMapping(Core& core, const std::unordered_map<Address, Entry>& dictionary, const std::unordered_map<std::string, Address>& name_to_address,
			uint16_t cob_id_, TransmissionType transmission_type_, std::chrono::milliseconds repeat_time_, const std::vector<Mapping>& mappings_)
	: cob_id(cob_id_),
		transmission_type(transmission_type_),
		repeat_time(repeat_time_),
		mappings(mappings_),
		m_core(core),
		m_dictionary(dictionary),
		m_name_to_address(name_to_address)
	{
		check_correctness();
	}

TransmitPDOMapping::~TransmitPDOMapping() {
	if (periodic_transmitter) {
		run_periodic_transmitter = false;
		periodic_transmitter->join();
	}
}


void TransmitPDOMapping::send() const {

	std::vector<uint8_t> data(8,0);
	size_t size=0;

	DEBUG_LOG("[TransmitPDOMapping::send] Sending transmit PDO with cob_id 0x"<<std::hex<<cob_id);

	for (const Mapping& mapping : mappings) {

		const std::string entry_name = Utils::escape(mapping.entry_name);
		const Entry& entry = m_dictionary.at(m_name_to_address.at(entry_name));
		const Value& value = entry.get_value();
		const std::vector<uint8_t> bytes = value.get_bytes();
		assert(mapping.offset+bytes.size() <= 8);

		DEBUG_LOG("[TransmitPDOMapping::send] Mapping:");
		DEBUG_DUMP(mapping.offset);
		DEBUG_DUMP(entry_name);
		DEBUG_DUMP_HEX(value);
		
		uint8_t count = 0;
		for (uint8_t i=mapping.offset; i<mapping.offset+bytes.size(); ++i) {
			data[i] = bytes[count++];
		}

		size = std::max(size, mapping.offset+bytes.size());

	}

	assert(size<=8 && "[TransmitPDOMapping::send] Malformed PDO mapping.");
	data.resize(size);
	m_core.pdo.send(cob_id, data);

}

void TransmitPDOMapping::check_correctness() const {

	unsigned size = 0;
	std::vector<bool> byte_mapped(8,false);

	for (const Mapping& mapping : mappings) {
		
		const std::string entry_name = Utils::escape(mapping.entry_name);

		if (m_name_to_address.count(entry_name) == 0) {
			throw dictionary_error(dictionary_error::type::unknown_entry, entry_name);
		}

		const Entry& entry = m_dictionary.at(m_name_to_address.at(entry_name));
		const uint8_t type_size = Utils::get_type_size(entry.type);

		if (mapping.offset+type_size > 8) {
			throw dictionary_error(dictionary_error::type::mapping_size, entry_name,
				"mapping.offset ("+std::to_string(mapping.offset)+") + type_size ("+std::to_string(type_size)+") > 8.");
		}

		for (uint8_t i=mapping.offset; i<mapping.offset+type_size; ++i) {
			if (byte_mapped[i]) {
				throw dictionary_error(dictionary_error::type::mapping_overlap, entry_name, "Byte index: "+std::to_string(i));
			}
			byte_mapped[i] = true;
		}

		size += type_size;

	}

	if (size > 8) {
		throw dictionary_error(dictionary_error::type::mapping_size, "",
				"total size ("+std::to_string(size)+") > 8.");
	}

}

} // end namespace kaco