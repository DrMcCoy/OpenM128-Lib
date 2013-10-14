/* onewire - Reading from / writing to the one-wire bus
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

#include <util/delay.h>

#include "onewire.h"

void onewire_reset(generic_io_t *wire) {
	generic_io_make_output(wire);
	generic_io_clear(wire);
	_delay_us(750);

	generic_io_make_input(wire, TRUE);

	while (generic_io_read(wire));
	while (!generic_io_read(wire));

	generic_io_make_output(wire);
}

uint8_t onewire_read(generic_io_t *wire) {
	uint8_t data = 0;

	for (uint8_t i = 0; i < 8; i++) {
		data >>= 1;

		generic_io_make_output(wire);
		generic_io_clear(wire);
		_delay_us(5);

		generic_io_make_input(wire, TRUE);
		_delay_us(15);

		if (generic_io_read(wire))
			data |= 0x80;
		else
			while (!generic_io_read(wire));
		_delay_us(60);

		generic_io_set(wire);
	}

	return data;
}

void onewire_write(generic_io_t *wire, uint8_t data) {
	generic_io_make_output(wire);
	for (uint8_t i = 0; i < 8; i++) {

		if (data & 0x01) {
			generic_io_clear(wire);
			_delay_us(5);

			generic_io_set(wire);
			_delay_us(85);
		} else {
			generic_io_clear(wire);
			_delay_us(90);

			generic_io_set(wire);
			_delay_us(5);
		}

		data >>= 1;
	}
}