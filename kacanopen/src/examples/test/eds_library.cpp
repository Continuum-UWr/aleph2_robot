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

#include "kacanopen/core/logger.h"
#include "kacanopen/master/eds_library.h"

#include <algorithm>
#include <unordered_map>

void print_dictionary(const std::unordered_map<kaco::Address, kaco::Entry>& map) {

	PRINT("\nHere is the dictionary:");

	using EntryRef = std::reference_wrapper<const kaco::Entry>;
	std::vector<EntryRef> entries;

	for (const auto& pair : map) {
		entries.push_back(std::ref(pair.second));
	}

	// sort by index and subindex
	std::sort(entries.begin(), entries.end(),
		[](const EntryRef& l, const EntryRef& r) { return l.get()<r.get(); });

	for (const auto& entry : entries) {
		entry.get().print();
	}

}

int main() {

	PRINT("This example loads dictionaries from the EDS library.");

	std::unordered_map<kaco::Address, kaco::Entry> dictionary;
	std::unordered_map<std::string, kaco::Address> name_to_address;
	kaco::EDSLibrary library(dictionary, name_to_address);
	bool success = library.lookup_library();

	if (!success) {
		ERROR("EDS library not found.");
		return EXIT_FAILURE;
	}

	success = library.load_default_eds(402);
	if (!success) {
		ERROR("load_default_eds(402) failed.");
	} else {
		print_dictionary(dictionary);
	}

	// This should fail.
	dictionary.clear();
	name_to_address.clear();
	success = library.load_default_eds(405);
	if (!success) {
		ERROR("load_default_eds(405) failed.");
	} else {
		print_dictionary(dictionary);
	}

	return EXIT_SUCCESS;

}
