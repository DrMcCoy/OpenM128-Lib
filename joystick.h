/* joystick - Reading the state of the joystick on the OpenM128 board
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

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include "types.h"

/** Type for all states a joystick can be in. */
typedef enum {
	kJoystickNone = 0x1F,  ///< Joystick at rest state, nothing pressed

	kJoystickDown = (0x1F & ~0x01), ///< Joystick pressed down

	kJoystickA = (0x1F & ~0x10), ///< Joystick pressed towards direction A
	kJoystickB = (0x1F & ~0x08), ///< Joystick pressed towards direction B
	kJoystickC = (0x1F & ~0x04), ///< Joystick pressed towards direction C
	kJoystickD = (0x1F & ~0x02), ///< Joystick pressed towards direction D

	kJoystickDownA = (0x1F & ~(0x10 | 0x01)), ///< Joystick pressed down and towards direction A
	kJoystickDownB = (0x1F & ~(0x08 | 0x01)), ///< Joystick pressed down and towards direction B
	kJoystickDownC = (0x1F & ~(0x04 | 0x01)), ///< Joystick pressed down and towards direction C
	kJoystickDownD = (0x1F & ~(0x02 | 0x01)), ///< Joystick pressed down and towards direction D
} joystick_state_t;

/** Initialize the joystick on the OpenM128 board. */
void joystick_init();

/** Read the state of the joystick on the OpenM128 board. */
joystick_state_t joystick_get();

#endif /* JOYSTICK_H_ */
