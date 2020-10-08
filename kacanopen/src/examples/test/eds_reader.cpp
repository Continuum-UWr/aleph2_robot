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

#include "ros/package.h"

#include "kacanopen/master/eds_reader.h"
#include "kacanopen/core/logger.h"

#include <tuple>
#include <algorithm>

// This is set by CMake...
//#define SHARE_SOURCE_PATH ...
//#define SHARE_INSTALLED_PATH ...

int main(int argc, char** argv) {

	PRINT("This example reads an EDS file and prints the resulting dictionary.");

	std::unordered_map<kaco::Address, kaco::Entry> dictionary;
	std::unordered_map<std::string, kaco::Address> name_to_address;
	kaco::EDSReader reader(dictionary, name_to_address);

	bool success = false;
	std::string path;

	if (argc>1 && argv[1]) {

		path = std::string(argv[1]);
		PRINT("Loading EDS file from "<<path);
		success = reader.load_file(path);

	} else {

		//path = SHARE_SOURCE_PATH "/example.eds";
		path = ros::package::getPath("kacanopen") + "/eds_library/example.eds";
		PRINT("Loading default EDS file from "<<path);
		success = reader.load_file(path);

		// if (!success) {

		// 	path = SHARE_INSTALLED_PATH "/example.eds";
		// 	PRINT("Another try: Loading default EDS file from "<<path);
		// 	success = reader.load_file(path);
		// }

	}

	if (!success) {
		ERROR("Loading file not successful. You can specify the path to the EDS file as command line argument.");
		return EXIT_FAILURE;
	}

	success = reader.import_entries();

	if (!success) {
		ERROR("Importing entries failed.");
		return EXIT_FAILURE;
	}

	PRINT("Here is the dictionary:");

	using EntryRef = std::reference_wrapper<const kaco::Entry>;
	std::vector<EntryRef> entries;

	for (const auto& pair : dictionary) {
		entries.push_back(std::ref(pair.second));
	}

	// sort by index and subindex
	std::sort(entries.begin(), entries.end(),
		[](const EntryRef& l, const EntryRef& r) { return l.get()<r.get(); });

	for (const auto& entry : entries) {
		entry.get().print();
	}

	return EXIT_SUCCESS;

}
