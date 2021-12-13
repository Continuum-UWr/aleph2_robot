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
 
#pragma once

#include <unordered_map>

#include "kacanopen/master/entry.h"
#include "kacanopen/master/address.h"

namespace kaco {

	class Device;

	/// This class provides access to KaCanOpen's EDS library.
	/// It manages device specific as well as generic CanOpen
	/// dictionaries.
	class EDSLibrary {

	public:

		/// Constructor.
		/// \param dictionary The dictionary, into which entries will be inserted.
		/// \param name_to_address Mapping from name to address in dictionary (to be created).
		EDSLibrary(std::unordered_map<Address, Entry>& dictionary, std::unordered_map<std::string, Address>& name_to_address);

		/// Finds EDS library on disk.
		/// \param path optional custom path to EDS library
		/// \returns true if successful
		bool lookup_library(std::string path = "");

		/// Loads mandatory dictionary entries defined in CiA 301 standard
		/// \returns true if successful
		bool load_mandatory_entries();

		/// Loads entries defined in generic CiA profile EDS files
		/// \param device_profile_number CiA standard profile number
		/// \returns true if successful
		bool load_default_eds(uint16_t device_profile_number);

		/// Loads entries defined in device specific EDS files proviced by manufacturers.
		/// \param vendor_id Vencor ID from identity object in dictionary (one of the mandatory entries)
		/// \param product_code Product code from identity object in dictionary (one of the mandatory entries)
		/// \param revision_number Revision number from identity object in dictionary (one of the mandatory entries)
		/// \returns true if successful
		/// \todo Remove this!
		bool load_manufacturer_eds_deprecated(uint32_t vendor_id, uint32_t product_code, uint32_t revision_number);

		/// Loads entries defined in device specific EDS files proviced by manufacturers.
		/// \param device Reference to the device (needed to fetch some information from the device)
		/// \returns true if successful
		bool load_manufacturer_eds(Device& device);

		/// Checks if lookup_library() was successful.
		/// \returns true if ready
		bool ready() const;

		/// Resets the dictionary and the name-address mapping.
		void reset_dictionary();

		/// Returns the path to the most recently loaded EDS file.
		std::string get_most_recent_eds_file_path() const;

	private:

		/// Enable debug logging.
		static const bool debug = false;

		/// Reference to the dictionary
		std::unordered_map<Address, Entry>& m_dictionary;

		/// Reference to the address-name mapping
		std::unordered_map<std::string, Address>& m_name_to_address;

		/// Path to the EDS library in filesystem. Set by lookup_library()
		std::string m_library_path;

		/// True, if lookup_library() was successful.
		bool m_ready;

		/// Stores the path to the most recently loaded EDS file.
		std::string most_recent_eds_file;

	};

} // end namespace kaco