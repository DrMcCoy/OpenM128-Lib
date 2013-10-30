/* pcf8574 - Using the PCF8574 8-bit I/O expander for the I²C bus
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

#include "openm128/pcf8574.h"
#include "openm128/i2c.h"

#define kPCF8574_DEVICE_MAX 8

static const uint8_t pcf8574_slave_address[kPCF8574_DEVICE_MAX] = { 0x20, 0x38 };

#define PCF8574_SLAVE_ADDRESS(type, id) (pcf8574_slave_address[type] | (id))

bool pcf8574_init(pcf8574_t *pcf8574, pcf8574_type_t type, uint8_t device_id, generic_io_t interrupt) {
	if ((type >= kPCF8574MAX) || (device_id >= kPCF8574_DEVICE_MAX))
		return FALSE;

	i2c_init();

	pcf8574->type      = type;
	pcf8574->device_id = device_id;
	pcf8574->interrupt = interrupt;

	return TRUE;
}

bool pcf8574_has_interrupt(const pcf8574_t *pcf8574) {
	return generic_io_read(&pcf8574->interrupt);
}

bool pcf8574_wait_interrupt(const pcf8574_t *pcf8574) {
	if (!generic_io_is_valid(&pcf8574->interrupt))
		return FALSE;

	while (!generic_io_read(&pcf8574->interrupt));
	return TRUE;
}

bool pcf8574_get(const pcf8574_t *pcf8574, uint8_t *data) {
	return i2c_read(PCF8574_SLAVE_ADDRESS(pcf8574->type, pcf8574->device_id), 1, data);
}

bool pcf8574_set(pcf8574_t *pcf8574, uint8_t data) {
	return i2c_write_byte(PCF8574_SLAVE_ADDRESS(pcf8574->type, pcf8574->device_id), data);
}
