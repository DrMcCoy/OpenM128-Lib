/* keys - Reading the keys on the Waveshare OpenM128 board
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

#include <avr/io.h>

#include "keys.h"

void keys_init(uint8_t mask) {
	if (mask & 0x01) {
		DDRD  &= ~0x40; // Set pin direction to input
		PORTD |=  0x40; // Enable pull-up resistor
	}
	if (mask & 0x02) {
		DDRD  &= ~0x80; // Set pin direction to input
		PORTD |=  0x80; // Enable pull-up resistor
	}
	if (mask & 0x04) {
		DDRE  &= ~0x40; // Set pin direction to input
		PORTE |=  0x40; // Enable pull-up resistor
	}
	if (mask & 0x08) {
		DDRE  &= ~0x80; // Set pin direction to input
		PORTE |=  0x80; // Enable pull-up resistor
	}
}

bool keys_get(uint8_t key) {
	switch (key) {
		case 0:
			return !(PIND & 0x40);
		case 1:
			return !(PIND & 0x80);
		case 2:
			return !(PINE & 0x40);
		case 3:
			return !(PINE & 0x80);
		default:
			return FALSE;
	}
	return FALSE;
}

uint8_t keys_get_mask(uint8_t mask) {
	uint8_t state = 0x00;

	for (uint8_t i = 0; i < 4; i++)
		if (mask & (1 << i))
			state |= keys_get(i) << i;

	return state;
}
