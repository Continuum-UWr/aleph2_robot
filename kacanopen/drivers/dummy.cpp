/*
 * Copyright (c) 2016 Thomas Keh
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

//-------------------------//
// This is a dummy driver. //
//-------------------------//

#include "kacanopen/core/message.h"
#include "kacanopen/core/logger.h"

/// CAN message type
using kaco::Message;

/// This struct contains C-strings for
/// busname and baudrate
struct CANBoard {

	/// Bus name
	const char * busname;

	/// Baudrate
	const char * baudrate;

};

/// Handle type which should represent the driver instance.
/// You can just some constant != 0 if only one instance is supported.
/// 0 is interpreted as failed initialization.
/// KaCanOpen uses just one instance.
using CANHandle = void*;

/// Initialize the driver and return some handle.
/// The board argument can be used for configuration.
extern "C" CANHandle canOpen_driver(CANBoard* board) {
	PRINT("canOpen_driver");
	(void) board;
	return (CANHandle) 1;
}

/// Destruct the driver.
/// Return 0 on success.
extern "C" int32_t canClose_driver(CANHandle handle) {
	PRINT("canClose_driver");
	(void) handle;
	return 0;
}

/// Receive a message.
/// This should be a blocking call and wait for any message.
/// Return 0 on success.
extern "C" uint8_t canReceive_driver(CANHandle handle, Message* message) {
	PRINT("canReceive_driver");
	(void) handle;
	(void) message;
	return 0;
}

/// Send a message
/// Return 0 on success.
extern "C" uint8_t canSend_driver(CANHandle handle, Message const* message) {
	PRINT("canSend_driver");
	(void) handle;
	(void) message;
	return 0;
}

/// Change the bus baudrate.
/// The baudrate is given as a C-string.
/// Supported values are 1M, 500K, 250K, 125K, 100K, 50K, 20K, 10K, 5K, and none.
/// Return 0 on success.
extern "C" uint8_t canChangeBaudRate_driver(CANHandle handle, char* baudrate) {
	PRINT("canChangeBaudRate_driver");
	(void) handle;
	(void) baudrate;
	return 0;
}