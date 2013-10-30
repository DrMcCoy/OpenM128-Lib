/* mag3110 - Reading from the MAG3110 magnetometer
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

#include "openm128/mag3110.h"
#include "openm128/i2c.h"

#define MAG3110_SLAVE_ADDRESS 0x0E

bool mag3110_init(void) {
	i2c_init();

	// Make sure the device ID is correct
	uint8_t device_id;
	if ((i2c_read8(MAG3110_SLAVE_ADDRESS, 0x07, 1, &device_id) != 1) || (device_id != 0xC4))
		return FALSE;

	// Raw mode
	if (!i2c_write8_byte(MAG3110_SLAVE_ADDRESS, 0x11, 0x20))
		return FALSE;

	// Continuous measurements, not fast mode, 20Hz output rate, 64 oversample, 1280Hz ADC rate, 900µA, 0.3µT rms noise
	if (!i2c_write8_byte(MAG3110_SLAVE_ADDRESS, 0x10, 0x11))
		return FALSE;

	return TRUE;
}

bool mag3110_has_measurement(void) {
	uint8_t status;
	if (i2c_read8(MAG3110_SLAVE_ADDRESS, 0x00, 1, &status) != 1)
		return FALSE;

	return !!(status & 0x40);
}

bool mag3110_wait_measurement(void) {
	uint8_t status = 0x00;

	while (!(status & 0x40))
		if (i2c_read8(MAG3110_SLAVE_ADDRESS, 0x00, 1, &status) != 1)
			return FALSE;

	return TRUE;
}

bool mag3110_get(int16_t *x, int16_t *y, int16_t *z) {
	uint8_t data[6];
	if (i2c_read8(MAG3110_SLAVE_ADDRESS, 0x01, 6, data) != 6)
		return FALSE;

	*x = (int16_t)((((uint16_t)data[0]) << 8) | data[1]);
	*y = (int16_t)((((uint16_t)data[2]) << 8) | data[3]);
	*z = (int16_t)((((uint16_t)data[4]) << 8) | data[5]);

	return TRUE;
}

bool mag3110_get_temperature(int8_t *temp) {
	return i2c_read8(MAG3110_SLAVE_ADDRESS, 0x0F, 1, (uint8_t *) temp);
}
