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

#include "openm128/at24cxx.h"

#define AT24CXX_BASE_ADDRESS (0xA0 >> 1)

/* Depending on the device type, the lower 3 bits of the slave address are used to either
   - specify an ID, so that several devices of the same type can be connected on the same bus
   - specify a memory page, so that more than 256 bytes can be addressed
   - a combination thereof
 */
#define AT24CXX_SLAVE_ADDRESS(type, id, daddr) (AT24CXX_BASE_ADDRESS | ((daddr) >> 8) | ((id) << (type)))

void at24cxx_init(void) {
	i2c_init();
}

bool at24cxx_is_address_valid(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address) {
	if (at24cxx >= kAT24CMAX)
		// Not a valid type
		return FALSE;

	// Depending on the type, 8, 4, 2 or 1 device can be on one bus
	if (device_id >= (1 << (3 - at24cxx)))
		// Not a valid device ID for the type
		return FALSE;

	if (address >= at24cxx_size(at24cxx))
		// Not a valid address for the type
		return FALSE;

	return TRUE;
}

uint16_t at24cxx_size(at24cxx_type_t at24cxx) {
	// Depending on the type, 256, 512, 1024 or 2048 bytes of memory are available
	return (1 << (8 + at24cxx));
}

bool at24cxx_poll(at24cxx_type_t at24cxx, uint8_t device_id, i2c_poll_t type) {
	if (!at24cxx_is_address_valid(at24cxx, device_id, 0))
		return FALSE;

	return i2c_poll(AT24CXX_SLAVE_ADDRESS(at24cxx, device_id, 0), type);
}

bool at24cxx_poll_wait(at24cxx_type_t at24cxx, uint8_t device_id, bool write) {
	if (!at24cxx_is_address_valid(at24cxx, device_id, 0))
		return FALSE;

	while (!at24cxx_poll(at24cxx, device_id, write));
	return TRUE;
}

bool at24cxx_read_byte(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint8_t *data) {
	if (!at24cxx_is_address_valid(at24cxx, device_id, address))
		return FALSE;

	return i2c_read(AT24CXX_SLAVE_ADDRESS(at24cxx, device_id, address), address & 0x00FF, 1, data) == 1;
}

bool at24cxx_write_byte(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint8_t data) {
	if (!at24cxx_is_address_valid(at24cxx, device_id, address))
		return FALSE;

	return i2c_write(AT24CXX_SLAVE_ADDRESS(at24cxx, device_id, address), address & 0x00FF, 1, &data) == 1;
}

uint16_t at24cxx_read(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint16_t n, uint8_t *data) {
	if (!at24cxx_is_address_valid(at24cxx, device_id, address))
		return 0;

	uint16_t processed = 0;
	while (n-- > 0) {
		if (!at24cxx_poll_wait(at24cxx, device_id, kI2CPollRead))
			break;
		if (!at24cxx_read_byte(at24cxx, device_id, address++, data++))
			break;

		processed++;
	}

	return processed;
}

uint16_t at24cxx_write(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint16_t n, uint8_t *data) {
	if (!at24cxx_is_address_valid(at24cxx, device_id, address))
		return 0;

	uint16_t processed = 0;
	while (n-- > 0) {
		if (!at24cxx_poll_wait(at24cxx, device_id, kI2CPollWrite))
			break;
		if (!at24cxx_write_byte(at24cxx, device_id, address++, *data++))
			break;

		processed++;
	}

	return processed;
}

bool at24cxx_read_word(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint16_t *data) {
	uint8_t tmp[2];
	if (at24cxx_read(at24cxx, device_id, address, 2, tmp) != 2)
		return FALSE;

	*data = ((uint16_t)tmp[0] << 8) | tmp[1];
	return TRUE;
}

bool at24cxx_write_word(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint16_t data) {
	uint8_t tmp[2];

	tmp[0] = (data >> 8) & 0xFF;
	tmp[1] =  data       & 0xFF;

	return at24cxx_write(at24cxx, device_id, address, 2, tmp) == 2;
}

bool at24cxx_read_dword(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint32_t *data) {
	uint8_t tmp[4];
	if (at24cxx_read(at24cxx, device_id, address, 4, tmp) != 4)
		return FALSE;

	*data = ((uint32_t)tmp[0] << 24) | ((uint32_t)tmp[1] << 16) | ((uint32_t)tmp[2] << 8) | tmp[3];
	return TRUE;
}

bool at24cxx_write_dword(at24cxx_type_t at24cxx, uint8_t device_id, uint16_t address, uint32_t data) {
	uint8_t tmp[4];

	tmp[0] = (data >> 24) & 0xFF;
	tmp[1] = (data >> 16) & 0xFF;
	tmp[2] = (data >>  8) & 0xFF;
	tmp[3] =  data        & 0xFF;

	return at24cxx_write(at24cxx, device_id, address, 4, tmp) == 4;
}
