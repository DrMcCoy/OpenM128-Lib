/* keypadtouch - Reading the state of the Waveshare capacitive touch keypad
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
#include <util/delay.h>

#include "openm128/keypadtouch.h"


/* Evil macro hackery to save me typing.
 * In short, reading whether a touch surface is active works this way:
 * First, we set the LOAD line to high, loading the capacitor; and wait until it is full.
 * Then, we set the LOAD line to low, letting the capacitor decharge.
 * We count the time it takes the capacitor to empty enough for the input line to read low.
 * If a finger is resting on the surface, the decharge time will be longer.
 *
 * To improve accuracy, a hardware timer could be used to count the decharge time.
 */
#define KEYPADTOUCH_READ(p, n, d) \
{\
	register uint16_t detectValue = 0;\
	PORTA |= (1 << PA1);\
	_delay_ms(1);\
	PORTA &= ~(1 << PA1);\
	while ((PIN##p & (1 << P##p##n)))\
		detectValue++;\
	d = detectValue;\
}

static void keypadtouch_read(uint16_t *data) {
	KEYPADTOUCH_READ(A, 3, data[0]);
	KEYPADTOUCH_READ(A, 4, data[1]);
	KEYPADTOUCH_READ(A, 5, data[2]);
	KEYPADTOUCH_READ(C, 5, data[3]);
	KEYPADTOUCH_READ(C, 4, data[4]);
	KEYPADTOUCH_READ(C, 3, data[5]);
	KEYPADTOUCH_READ(C, 2, data[6]);
	KEYPADTOUCH_READ(C, 1, data[7]);
}


void keypadtouch_init(keypadtouch_t *keypad) {
	// The key and wheel lines are input, load and shield output
	DDRA  = 0x06;
	DDRC  = 0x01;

	// Set the two shield output lines to high
	PORTA = 0x04;
	PORTC = 0x01;

	keypadtouch_recalibrate(keypad);
}

void keypadtouch_recalibrate(keypadtouch_t *keypad) {
	keypadtouch_read(keypad->zero);
}

#define KEYPADTOUCH_ERROR_RANGE 1
#define KEYPADTOUCH_SURFACE_ACTIVE(x) ((data[(x)] > (keypad->zero[(x)] + KEYPADTOUCH_ERROR_RANGE)))
keypadtouch_state_t keypadtouch_get(keypadtouch_t *keypad) {
	uint16_t data[8];
	keypadtouch_read(data);

	keypadtouch_state_t state;

	state.key1 = KEYPADTOUCH_SURFACE_ACTIVE(0);
	state.key2 = KEYPADTOUCH_SURFACE_ACTIVE(1);
	state.key3 = KEYPADTOUCH_SURFACE_ACTIVE(2);

	/* Detect the slider value. Valid states are:
	 * - On no slider surface
	 * - Directly on one slider surface
	 * - Between two surfaces
	 */
	state.slider = 0;
	for (uint8_t i = 0; i < 5; i++) {
		// Find the lowest surface we're on
		if (KEYPADTOUCH_SURFACE_ACTIVE(3 + i)) {
			state.slider = 2 * i + 1;

		// Offset for every next surface we're on as well.
		// This counts being on three surfaces as being on the middle surface.
		for (uint8_t j = i + 1; j < 5; j++)
			if (KEYPADTOUCH_SURFACE_ACTIVE(3 + j))
				state.slider++;

			break;
		}
	}

	return state;
}
