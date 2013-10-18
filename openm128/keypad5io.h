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

#ifndef KEYPAD5IO_H_
#define KEYPAD5IO_H_

#include "openm128/types.h"
#include "openm128/generic_io.h"

typedef enum {
	kKeypad5ioKeyNone,
	kKeypad5ioJoystickDown,
	kKeypad5ioJoystickA,
	kKeypad5ioJoystickB,
	kKeypad5ioJoystickC,
	kKeypad5ioJoystickD,
	kKeypad5ioJoystickDownA,
	kKeypad5ioJoystickDownB,
	kKeypad5ioJoystickDownC,
	kKeypad5ioJoystickDownD,
	kKeypad5ioKey1,
	kKeypad5ioKey2,
	kKeypad5ioKey3,
	kKeypad5ioKey4,
	kKeypad5ioKey5,
	kKeypad5ioKey6,
	kKeypad5ioKey7,
	kKeypad5ioKey8,
	kKeypad5ioKey9,
	kKeypad5ioKey10
} keypad5io_key_t;

typedef struct {
	generic_io_t io[5];
} keypad5io_t;

void keypad5io_init(keypad5io_t *keypad, generic_io_t io1,
                                         generic_io_t io2,
                                         generic_io_t io3,
                                         generic_io_t io4,
                                         generic_io_t io5);

keypad5io_key_t keypad5io_get(keypad5io_t *keypad);

#endif /* KEYPAD5IO_H_ */
