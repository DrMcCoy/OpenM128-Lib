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

#include "keypad4x4.h"

void keypad4x4_init(keypad4x4_t *keypad, generic_io_t k0,
                                         generic_io_t k1,
                                         generic_io_t k2,
                                         generic_io_t k3,
                                         generic_io_t k4,
                                         generic_io_t k5,
                                         generic_io_t k6,
                                         generic_io_t k7) {

	keypad->k[0] = k0;
	keypad->k[1] = k1;
	keypad->k[2] = k2;
	keypad->k[3] = k3;
	keypad->k[4] = k4;
	keypad->k[5] = k5;
	keypad->k[6] = k6;
	keypad->k[7] = k7;
}

keypad4x4_key_t keypad4x4_get(keypad4x4_t *keypad) {
	return kKeypad4x4KeyNone;
}