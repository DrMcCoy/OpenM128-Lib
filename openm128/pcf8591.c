/* pcf8591 - Using the PCF8591 A/D and D/A converter for the I²C bus
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

#include <string.h>

#include "openm128/pcf8591.h"
#include "openm128/i2c.h"

#define kPCF8591_DEVICE_MAX  8

static const uint8_t pcf8591_channel_max[kPCF8591InputMAX] = { 4, 3, 3, 2 };

#define PCF8591_BASE_ADDRESS (0x90 >> 1)
#define PCF8591_SLAVE_ADDRESS(id) (PCF8591_BASE_ADDRESS | (id))

bool pcf8591_init(uint8_t device_id) {
	if (device_id >= kPCF8591_DEVICE_MAX)
		return FALSE;

	i2c_init();

	return pcf8591_set(device_id, 0);
}

bool pcf8591_get(uint8_t device_id, pcf8591_input_mode_t mode, uint8_t channel, uint8_t *data) {
	if ((device_id >= kPCF8591_DEVICE_MAX) || (mode >= kPCF8591InputMAX) || (channel >= pcf8591_channel_max[mode]))
		return FALSE;

	// Keep output enabled, select mode and channel
	uint8_t control = 0x40 | (mode << 4) | channel;

	if (!i2c_write_byte(PCF8591_SLAVE_ADDRESS(device_id), control))
		return FALSE;

	// Read two values. The first is the result from the last conversion, the second what we actually want
	uint8_t values[2];
	if (i2c_read(PCF8591_SLAVE_ADDRESS(device_id), 2, values) != 2)
		return FALSE;

	*data = values[1];
	return TRUE;
}

bool pcf8591_get_all(uint8_t device_id, pcf8591_input_mode_t mode, uint8_t *data) {
	if ((device_id >= kPCF8591_DEVICE_MAX) || (mode >= kPCF8591InputMAX))
		return FALSE;

	// Keep output enabled, select mode and first channel in auto-increment mode
	uint8_t control = 0x44 | (mode << 4);

	if (!i2c_write_byte(PCF8591_SLAVE_ADDRESS(device_id), control))
		return FALSE;

	// Read one value more than channels. The first one is the result from the previous conversion, the latter what we want
	uint8_t values[5];
	if (i2c_read(PCF8591_SLAVE_ADDRESS(device_id), pcf8591_channel_max[mode] + 1, values) != (uint16_t)(pcf8591_channel_max[mode] + 1))
		return FALSE;

	memcpy(data, values + 1, pcf8591_channel_max[mode]);
	return TRUE;
}

bool pcf8591_set(uint8_t device_id, uint8_t data) {
	if (device_id >= kPCF8591_DEVICE_MAX)
		return FALSE;

	uint8_t control[2];

	control[0] = 0x40; // Enable output, reset input
	control[1] = data;

	return i2c_write(PCF8591_SLAVE_ADDRESS(device_id), 2, control) == 2;
}
