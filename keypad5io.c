/* keypad5io - Reading the keys of the Waveshare 5 IO keypad
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

#include "keypad5io.h"

void keypad5io_init(keypad5io_t *keypad, generic_io_t io1,
                                         generic_io_t io2,
                                         generic_io_t io3,
                                         generic_io_t io4,
                                         generic_io_t io5) {

	keypad->io[0] = io1;
	keypad->io[1] = io2;
	keypad->io[2] = io3;
	keypad->io[3] = io4;
	keypad->io[4] = io5;
}

/** Reading the 5 IO keypad works like this:
  *
  * - First, we configure all pins to input, and read if one bit is not set.
  * - If so, the joystick was pressed/moved.
  * - Otherwise, we configure the first pin as output (and pull the line down) and the rest as input.
  * - If one of the read bits is not set, one of K1, K2, K3, K4 was pressed.
  * - Otherwise, we configure the first 2 pins as output (and pull the lines down) and the rest as input.
  * - If one of the read bits is not set, one of K5, K6, K7 was pressed.
  * - Otherwise, we configure the first 3 pins as output (and pull the lines down) and the rest as input.
  * - If one of the read bits is not set, one of K8, K9 was pressed.
  * - Otherwise, we configure the first 4 pins as output (and pull the lines down) and the rest as input.
  * - If the read bit is not set, K10 was pressed.
  * - Otherwise, nothing was pressed.
  */

static keypad5io_key_t keypad5io_read_joystick(keypad5io_t *keypad) {
	generic_io_make_input(&keypad->io[0], TRUE);
	generic_io_make_input(&keypad->io[1], TRUE);
	generic_io_make_input(&keypad->io[2], TRUE);
	generic_io_make_input(&keypad->io[3], TRUE);
	generic_io_make_input(&keypad->io[4], TRUE);

	uint8_t value = generic_io_read_multi(keypad->io, 5);

	switch (value) {
		case 0x1B:
			return kKeypad5ioJoystickDown;
		case 0x1D:
			return kKeypad5ioJoystickA;
		case 0x0F:
			return kKeypad5ioJoystickB;
		case 0x17:
			return kKeypad5ioJoystickC;
		case 0x1E:
			return kKeypad5ioJoystickD;
		case 0x19:
			return kKeypad5ioJoystickDownA;
		case 0x0B:
			return kKeypad5ioJoystickDownB;
		case 0x13:
			return kKeypad5ioJoystickDownC;
		case 0x1A:
			return kKeypad5ioJoystickDownD;
	}

	return kKeypad5ioKeyNone;
}

static keypad5io_key_t keypad5io_read_keyLine1(keypad5io_t *keypad) {
	generic_io_make_output(&keypad->io[0]);

	generic_io_clear(&keypad->io[0]);

	generic_io_make_input(&keypad->io[1], TRUE);
	generic_io_make_input(&keypad->io[2], TRUE);
	generic_io_make_input(&keypad->io[3], TRUE);
	generic_io_make_input(&keypad->io[4], TRUE);

	uint8_t value = generic_io_read_multi(keypad->io + 1, 4);

	if (!(value & 0x01))
		return kKeypad5ioKey1;
	if (!(value & 0x02))
		return kKeypad5ioKey2;
	if (!(value & 0x04))
		return kKeypad5ioKey3;
	if (!(value & 0x08))
		return kKeypad5ioKey4;

	return kKeypad5ioKeyNone;
}

static keypad5io_key_t keypad5io_read_keyLine2(keypad5io_t *keypad) {
	generic_io_make_output(&keypad->io[0]);
	generic_io_make_output(&keypad->io[1]);

	generic_io_clear(&keypad->io[0]);
	generic_io_clear(&keypad->io[1]);

	generic_io_make_input(&keypad->io[2], TRUE);
	generic_io_make_input(&keypad->io[3], TRUE);
	generic_io_make_input(&keypad->io[4], TRUE);

	uint8_t value = generic_io_read_multi(keypad->io + 2, 3);

	if (!(value & 0x01))
		return kKeypad5ioKey5;
	if (!(value & 0x02))
		return kKeypad5ioKey6;
	if (!(value & 0x04))
		return kKeypad5ioKey7;

	return kKeypad5ioKeyNone;
}

static keypad5io_key_t keypad5io_read_keyLine3(keypad5io_t *keypad) {
	generic_io_make_output(&keypad->io[0]);
	generic_io_make_output(&keypad->io[1]);
	generic_io_make_output(&keypad->io[2]);

	generic_io_clear(&keypad->io[0]);
	generic_io_clear(&keypad->io[1]);
	generic_io_clear(&keypad->io[2]);

	generic_io_make_input(&keypad->io[3], TRUE);
	generic_io_make_input(&keypad->io[4], TRUE);

	uint8_t value = generic_io_read_multi(keypad->io + 3, 2);

	if (!(value & 0x01))
		return kKeypad5ioKey8;
	if (!(value & 0x02))
		return kKeypad5ioKey9;

	return kKeypad5ioKeyNone;
}

static keypad5io_key_t keypad5io_read_keyLine4(keypad5io_t *keypad) {
	generic_io_make_output(&keypad->io[0]);
	generic_io_make_output(&keypad->io[1]);
	generic_io_make_output(&keypad->io[2]);
	generic_io_make_output(&keypad->io[3]);

	generic_io_clear(&keypad->io[0]);
	generic_io_clear(&keypad->io[1]);
	generic_io_clear(&keypad->io[2]);
	generic_io_clear(&keypad->io[3]);

	generic_io_make_input(&keypad->io[4], TRUE);

	uint8_t value = generic_io_read_multi(keypad->io + 4, 1);

	if (!(value & 0x01))
		return kKeypad5ioKey10;

	return kKeypad5ioKeyNone;

}

keypad5io_key_t keypad5io_get(keypad5io_t *keypad) {
	keypad5io_key_t key;

	key = keypad5io_read_joystick(keypad);
	if (key != kKeypad5ioKeyNone)
		return key;
	key = keypad5io_read_keyLine1(keypad);
	if (key != kKeypad5ioKeyNone)
		return key;
	key = keypad5io_read_keyLine2(keypad);
	if (key != kKeypad5ioKeyNone)
		return key;
	key = keypad5io_read_keyLine3(keypad);
	if (key != kKeypad5ioKeyNone)
		return key;
	key = keypad5io_read_keyLine4(keypad);
	if (key != kKeypad5ioKeyNone)
		return key;

	return kKeypad5ioKeyNone;
}