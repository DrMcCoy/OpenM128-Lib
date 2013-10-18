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

#ifndef KEYPADTOUCH_H_
#define KEYPADTOUCH_H_

#include "openm128/types.h"
#include "openm128/generic_io.h"

typedef struct {
	uint16_t zero[8];
} keypadtouch_t;

typedef struct {
	bool key1; ///< Is Key 1 active?
	bool key2; ///< Is Key 2 active?
	bool key3; ///< Is Key 3 active?
} keypadtouch_state_t;

/** Initialize and calibrate the capacitive touch keypad. */
void keypadtouch_init(keypadtouch_t *keypad);

/** Re-calibrate the capacitive touch keypad. The keys are assumed to be not active. */
void keypadtouch_recalibrate(keypadtouch_t *keypad);

/** Return the current state of the capacitive touch keypad. */
keypadtouch_state_t keypadtouch_get(keypadtouch_t *keypad);

#endif /* KEYPADTOUCH_H_ */
