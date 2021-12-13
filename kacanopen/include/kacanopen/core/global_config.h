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

#include <cstddef>

namespace kaco {

	/// This class holds global configuration
	class Config {

	public:

		/// Timeout in milliseconds when waiting for an SDO response.
		static size_t sdo_response_timeout_ms;
		
		/// Number of repetitions when an SDO timeout occurs in SDO download/upload
		static size_t repeats_on_sdo_timeout;

		/// If this is set to true, EDSReader will mark all entries as generic (Entry::is_generic)
		/// This is used internally. Do not modify this unless you know what you do!
		static bool eds_reader_mark_entries_as_generic;
		
		/// If this is set to true, EDSReader won't create any entries, but just adds
		/// name-to-address mappings for the (generic) entry names.
		static bool eds_reader_just_add_mappings;
		
		/// If this is set to true, EDSLibrary will clear dictionary and me-to-address mappings
		/// before loading an EDS file.
		static bool eds_library_clear_dictionary;

	};

} // end namespace kaco
