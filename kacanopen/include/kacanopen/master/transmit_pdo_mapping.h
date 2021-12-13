/*
 * Copyright (c) 2015-2016, Thomas Keh
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

#include <string>
#include <vector>
#include <chrono>
#include <unordered_map>
#include <thread>
#include <memory>

#include "kacanopen/master/mapping.h"
#include "kacanopen/master/types.h"
#include "kacanopen/master/address.h"

namespace kaco {

	// forward declarations
	class Core;
	class Entry;

	/// This class represents a mapping from one or more
	/// dictionary entries to one transmit PDO, which will
	/// be sent by an instance of this class repeatedly.
	class TransmitPDOMapping {

	public:

		/// Constructor.
		/// \param core Reference to the Core instance (needed to send the PDO).
		/// \param dictionary Reference to the object dictionary.
		/// \param name_to_address Reference to name-address mapping.
		/// \param cob_id_ COB-ID of the PDO
		/// \param transmission_type_ Transmission type
		/// \param repeat_time_ Send repeat time , in case transmission_type_==TransmissionType::PERIODIC
		/// \param mappings_ Mapped entries with offset (see Mapping class)
		/// \throws dictionary_error if entry does not exist or mappings overlap (among others)
		TransmitPDOMapping(Core& core, const std::unordered_map<Address, Entry>& dictionary, const std::unordered_map<std::string, Address>& name_to_address, uint16_t cob_id_,
			TransmissionType transmission_type_, std::chrono::milliseconds repeat_time_, const std::vector<Mapping>& mappings_);

		/// Copy constructor deleted.
		TransmitPDOMapping(const TransmitPDOMapping&) = delete;

		/// Move constructor deleted.
		TransmitPDOMapping(TransmitPDOMapping&&) = delete;

		/// Stops the transmitter thread if there is one.
		~TransmitPDOMapping();

		/// COB-ID of the PDO
		uint16_t cob_id;

		/// Transmission type
		TransmissionType transmission_type;

		/// Send repeat time
		std::chrono::milliseconds repeat_time;

		/// Mapped entries with offset (see Mapping class)
		std::vector<Mapping> mappings;

		/// The transmitter thread
		/// \note This is a unique pointer because transmitter is optional.
		std::unique_ptr<std::thread> periodic_transmitter{nullptr};

		bool run_periodic_transmitter{false};

		/// Sends the PDO
		void send() const;

	private:

		/// \throws dictionary_error
		void check_correctness() const;

		static const bool debug = false;

		Core& m_core;

		/// Reference to the dictionary
		const std::unordered_map<Address, Entry>& m_dictionary;

		/// Reference to the address-name mapping
		const std::unordered_map<std::string, Address>& m_name_to_address;

	};

} // end namespace kaco