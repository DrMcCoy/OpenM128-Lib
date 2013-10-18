/* ds1820 - Reading temperature data from the DS18B20 one-wire sensor
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

#include "ds18b20.h"

int16_t ds18b20_get(generic_io_t *wire) {
	onewire_reset(wire);
	onewire_write(wire, 0xCC);
	onewire_write(wire, 0x44);

	onewire_reset(wire);
	onewire_write(wire, 0xCC);
	onewire_write(wire, 0xBE);

	// The lower 4 bits of temp1 are the fractional part of the temperature
	// The upper 4 bits of temp1 and the lower 4 bits of temp2 are the integer part of the temperature

	const uint8_t temp1 = onewire_read(wire);
	const uint8_t temp2 = onewire_read(wire);

	const int16_t tempI = ((int16_t)((int8_t)((temp2 << 4) | (temp1 >> 4)))) * 100;
	const int16_t tempF = (((int16_t)(temp1 & 0x0F)) * 100) >> 4;

	return tempI + tempF;
}
