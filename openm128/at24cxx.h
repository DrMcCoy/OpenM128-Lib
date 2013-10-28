/* at24cxx - Reading from / writing to the AT24CXX EEPROM
 *
 * Copyright (c) 2013, Sven Hesse <drmccoy@drmccoy.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef AT24CXX_H_
#define AT24CXX_H_

#include "openm128/types.h"
#include "openm128/i2c.h"

/** The type of AT24CXX device / size of the memory. */
typedef enum {
	kAT24C02 = 0, ///< AT24C02:  2 kilobit ( 256 byte) of memory. 8 devices can be on one bus.
	kAT24C04 = 1, ///< AT24C04:  4 kilobit ( 512 byte) of memory. 4 devices can be on one bus.
	kAT24C08 = 2, ///< AT24C08:  8 kilobit (1024 byte) of memory. 2 devices can be on one bus.
	kAT24C16 = 3, ///< AT24C16: 16 kilobit (2048 byte) of memory. 1 device  can be on one bus.

	kAT24CMAX     ///< No real type; for range checks.
} at24cxx_type_t;


/** Initialize the I²C for accessing AT24CXX devices of the specified type. */
void at24cxx_init(void);

/** Is this combination of type, device ID and address valid? */
bool at24cxx_is_address_valid(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address);

/** Returns the size of the memory in bytes on this device type. */
uint16_t at24cxx_size(at24cxx_type_t at24cxx);

/** Query whether a specific AT24CXX device is ready to perform writing/reading.
 *
 *  @param at24cxx   The type of the AT24CXX device to poll.
 *  @param device_id The ID of the device to poll.
 *  @param type      The type of the operation to poll for.
 *
 *  @return TRUE if the device is ready to perform the specified operation.
 */
bool at24cxx_poll(at24cxx_type_t at24cxx, uint8_t device_id, i2c_poll_t type);

/** Wait until a specific AT24CXX device is ready to perform writing/reading.
 *
 *  See at24cxx_poll().
 *
 *  @return TRUE if the device is ready to perform the specified operation, FALSE on invalid device.
 */
bool at24cxx_poll_wait(at24cxx_type_t at24cxx, uint8_t device_id, bool write);

/** Read several bytes from an AT24CXX device.
 *
 *  @param at24cxx   The type of the AT24CXX device to read from.
 *  @param device_id The ID of the device to read from.
 *  @param address   The address from where to start reading.
 *  @param n         The number of bytes to read.
 *  @param data      The read data will be stored here.
 *
 *  @return The number of bytes successfully read.
 */
uint16_t at24cxx_read(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint16_t n, uint8_t *data);

/** Write several bytes to an AT24CXX device.
 *
 *  @param at24cxx   The type of the AT24CXX device to write to.
 *  @param device_id The ID of the device to start writing to.
 *  @param address   The address to write to.
 *  @param n         The number of bytes to write.
 *  @param data      The data to write.
 *
 *  @return The number of bytes successfully written.
 */
uint16_t at24cxx_write(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint16_t n, uint8_t *data);

/** Read a byte from an AT24CXX device.
 *
 *  @param at24cxx   The type of the AT24CXX device to read from.
 *  @param device_id The ID of the device to read from.
 *  @param address   The address from where to read.
 *  @param data      The read data will be stored here.
 *
 *  @return TRUE if the reading was successful, FALSE otherwise.
 */
bool at24cxx_read_byte(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint8_t *data);

/** Write a byte to an AT24CXX device.
 *
 *  @param at24cxx   The type of the AT24CXX device to write to.
 *  @param device_id The ID of the device to write to.
 *  @param address   The address to write to.
 *  @param data      The data to write.
 *
 *  @return TRUE if the writing was successful, FALSE otherwise.
 */
bool at24cxx_write_byte(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint8_t data);

/** Read a word (16 bit) from an AT24CXX device, MSB first. */
bool at24cxx_read_word(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint16_t *data);

/** Write a word (16 bit) to an AT24CXX device, MSB first. */
bool at24cxx_write_word(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint16_t data);

/** Read a double-word (32 bit) from an AT24CXX device, MSB first. */
bool at24cxx_read_dword(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint32_t *data);

/** Write a double-word (32 bit) to an AT24CXX device, MSB first. */
bool at24cxx_write_dword(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint32_t data);

#endif /* AT24CXX_H_ */
