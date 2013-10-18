/* keypad4x4 - Reading the keys of the Waveshare 4x4 matrix keypad
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

#ifndef KEYPAD4X4_H_
#define KEYPAD4X4_H_

#include "types.h"
#include "generic_io.h"

typedef enum {
	kKeypad4x4KeyNone   = 0xFF,
	kKeypad4x4Key0      =   13,
	kKeypad4x4Key1      =    0,
	kKeypad4x4Key2      =    1,
	kKeypad4x4Key3      =    2,
	kKeypad4x4Key4      =    4,
	kKeypad4x4Key5      =    5,
	kKeypad4x4Key6      =    6,
	kKeypad4x4Key7      =    8,
	kKeypad4x4Key8      =    9,
	kKeypad4x4Key9      =   10,
	kKeypad4x4KeyEnter  =   12,
	kKeypad4x4KeyEscape =   14,
	kKeypad4x4KeyPower  =   15,
	kKeypad4x4KeyStop   =    3,
	kKeypad4x4KeyGo     =    7,
	kKeypad4x4KeyLock   =   11
} keypad4x4_key_t;

typedef struct {
	generic_io_t k[8];
} keypad4x4_t;

void keypad4x4_init(keypad4x4_t *keypad, generic_io_t k0,
                                         generic_io_t k1,
                                         generic_io_t k2,
                                         generic_io_t k3,
                                         generic_io_t k4,
                                         generic_io_t k5,
                                         generic_io_t k6,
                                         generic_io_t k7);

keypad4x4_key_t keypad4x4_get(keypad4x4_t *keypad);

#endif /* KEYPAD4X4_H_ */
