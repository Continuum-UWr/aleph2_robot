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
#include "kacanopen/core/global_config.h"

#include "kacanopen/master/eds_library.h"
#include "kacanopen/master/eds_reader.h"
#include "kacanopen/master/device.h"
#include "kacanopen/master/types.h"
#include "kacanopen/master/value.h"

#include <vector>
#include <string>
#include <unordered_map>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp> // property_tree
#include <boost/property_tree/json_parser.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"

// This is set by CMake...
//#define SHARE_SOURCE_PATH ...
//#define SHARE_INSTALLED_PATH ...

namespace kaco {

	namespace fs = boost::filesystem;

	EDSLibrary::EDSLibrary(std::unordered_map<Address, Entry>& dictionary, std::unordered_map<std::string, Address>& name_to_address)
		: m_dictionary(dictionary), m_name_to_address(name_to_address), m_ready(false)
		{ }

	bool EDSLibrary::lookup_library(std::string path) {

		m_ready = false;

		std::vector<std::string> paths;
		if (!path.empty()) {
			paths.push_back(path+"/eds_files.json");
		}

		paths.push_back(ament_index_cpp::get_package_share_directory("kacanopen") + "/eds_library/eds_files.json");

		// paths.push_back(SHARE_SOURCE_PATH "/eds_library/eds_files.json");
		// paths.push_back(SHARE_INSTALLED_PATH "/eds_library/eds_files.json");
		// paths.push_back("/usr/local/share/kacanopen/eds_library/eds_files.json");
		// paths.push_back("/usr/share/kacanopen/eds_library/eds_files.json");
		// TODO: typical windows / mac osx paths?
		// TODO: environment variable

		bool success = false;
		for (const std::string& path : paths) {
			if (fs::exists(path)) {
				m_library_path = fs::path(path).parent_path().string();
				DEBUG_LOG("[EDSLibrary::lookup_library] Found EDS library in "<<m_library_path);
				success = true;
				break;
			}
		}

		if (!success) {
			DEBUG_LOG("[EDSLibrary::lookup_library] Could not find EDS library. You should pass a custum path or check your local installation.")
			return false;
		}

		m_ready = true;
		return true;
	}

	bool EDSLibrary::load_mandatory_entries() {
		return load_default_eds(301);
	}

	bool EDSLibrary::load_default_eds(uint16_t device_profile_number) {

		assert(m_ready);

		std::string path = m_library_path + "/CiA_profiles/"+std::to_string(device_profile_number)+".eds";
		if (!fs::exists(path)) {
			DEBUG_LOG("[EDSLibrary::load_default_eds] Default EDS file not available: "<<path);
			return false;
		}

		
		if (Config::eds_library_clear_dictionary) {
			reset_dictionary();
		}

		EDSReader reader(m_dictionary, m_name_to_address);
		bool success = reader.load_file(path);

		if (!success) {
			ERROR("[EDSLibrary::load_default_eds] Loading file not successful.");
			return false;
		}

		DEBUG_LOG("[EDSLibrary::load_default_eds] Found EDS file: "<<path);
		success = reader.import_entries();
		most_recent_eds_file = path;

		if (!success) {
			ERROR("[EDSLibrary::load_default_eds] Importing entries failed.");
			return false;
		}

		return true;

	}

	bool EDSLibrary::load_manufacturer_eds(Device& device) {
		
		boost::property_tree::ptree eds_files;
		boost::property_tree::json_parser::read_json(m_library_path+"/eds_files.json", eds_files);
		std::unordered_map<std::string,Value> cache;

		for (const auto& level0 : eds_files) {

			const std::string& filename = level0.second.get<std::string>("file");
			DEBUG_LOG("Testing if "<<filename<<" fits.");
			const boost::property_tree::ptree& matches = level0.second.get_child("match");
			bool fits = true;

			for (const auto& level1 : matches) {
				
				const std::string& field = level1.first;
				const std::string& value_expected = level1.second.get_value<std::string>();
				bool has_field = false;

				if (cache.count(field)>0) {
					// field is already in cache
					if (cache[field].type != Type::invalid) {
						has_field = true;
					}
				} else {
					try {
						const Value& temp = device.get_entry(field);
						cache[field] = temp;
						has_field = true;
					} catch (const canopen_error& err){
						DEBUG_LOG("  ("<<err.what()<<")");
						cache[field] = Value(); // invalid value
					}
				}

				if (has_field) {
					const std::string& value = cache[field].to_string().substr(0,value_expected.length());
					if (value_expected == value) {
						DEBUG_LOG("  "<<field<<": "<<value<<" == "<<value_expected<<" (expected) -> continue.");
					} else {
						DEBUG_LOG("  "<<field<<": "<<value<<" != "<<value_expected<<" (expected) -> break.");
						fits = false;
					}
				} else {
					DEBUG_LOG("  Field does not exist -> break...");
					fits = false;
				}

				if (!fits) {
					break;
				}

			}

			if (fits) {
				
				DEBUG_LOG("  "<<filename<<" fits.");
				const std::string path = m_library_path + "/"+filename;
				assert(fs::exists(path));
				
				if (Config::eds_library_clear_dictionary) {
					reset_dictionary();
				}

				EDSReader reader(m_dictionary, m_name_to_address);
				bool success = reader.load_file(path);

				if (!success) {
					ERROR("[EDSLibrary::load_manufacturer_eds] Loading file not successful.");
					return false;
				}

				success = reader.import_entries();
				most_recent_eds_file = path;

				if (!success) {
					ERROR("[EDSLibrary::load_manufacturer_eds] Importing entries failed.");
					return false;
				}

				return true;

			} else {
				DEBUG_LOG("  "<<filename<<" does not fit.");
			}

		}

		DEBUG_LOG("No suitable manufacturer EDS file found.");
		return false;

	}

	bool EDSLibrary::load_manufacturer_eds_deprecated(uint32_t vendor_id, uint32_t product_code, uint32_t revision_number) {
		assert(m_ready);

		// check if there is an EDS file for this revision
		std::string path = m_library_path + "/"+std::to_string(vendor_id)+"/"+std::to_string(product_code)+"."+std::to_string(revision_number)+".eds";
		if (!fs::exists(path)) {
			// check if there is a generic EDS file for product
			path = m_library_path + "/"+std::to_string(vendor_id)+"/"+std::to_string(product_code)+".eds";
			if (!fs::exists(path)) {
				DEBUG_LOG("[EDSLibrary::load_manufacturer_eds] Manufacturer device specific EDS file not available: "<<path);
				return false;
			}
		}
		
		DEBUG_LOG("[EDSLibrary::load_manufacturer_eds] Found manufacturer EDS: "<<path);

		if (Config::eds_library_clear_dictionary) {
			reset_dictionary();
		}

		EDSReader reader(m_dictionary, m_name_to_address);
		bool success = reader.load_file(path);
		most_recent_eds_file = path;

		if (!success) {
			ERROR("[EDSLibrary::load_manufacturer_eds] Loading file not successful.");
			return false;
		}

		success = reader.import_entries();

		if (!success) {
			ERROR("[EDSLibrary::load_manufacturer_eds] Importing entries failed.");
			return false;
		}

		return true;
		
	}

	bool EDSLibrary::ready() const {
		return m_ready;
	}
	
	void EDSLibrary::reset_dictionary() {
		m_dictionary.clear();
		m_name_to_address.clear();
	}
	
	std::string EDSLibrary::get_most_recent_eds_file_path() const {
		return most_recent_eds_file;
	}

} // end namespace kaco