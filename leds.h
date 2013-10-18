/* leds - Set the LEDs on the Waveshare OpenM128 board
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

#ifndef LEDS_H_
#define LEDS_H_

#include "types.h"

/** Utility macro to convert 8 separate booleans (for each LED) into a LED bitmask. */
#define LEDS_MASK(LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7) (((LED0) ? 0x01 : 0x00) |\
                                                                   ((LED1) ? 0x02 : 0x00) |\
                                                                   ((LED2) ? 0x04 : 0x00) |\
                                                                   ((LED3) ? 0x08 : 0x00) |\
                                                                   ((LED4) ? 0x10 : 0x00) |\
                                                                   ((LED5) ? 0x20 : 0x00) |\
                                                                   ((LED6) ? 0x40 : 0x00) |\
                                                                   ((LED7) ? 0x80 : 0x00))

/** Initialize the LEDs on the OpenM128 boards.
 *
 *  @param mask A mask to specify which LEDs should be initialized.
 *              If bit n (0-7) is set, LED n will be initialized.
 *              If bit n (0-7) is not set, LED n will not be initialized.
 */
void leds_init(uint8_t mask);

/** Set the state of one LED.
 *
 *  @param led Which LED (0-7) should be set?
 *  @param on  Should the LED be switched on?
 */
void leds_set(uint8_t led, bool on);

/** Set the state of several LEDs according to a mask.
 *
 *  @param mask A mask to specific which LEDs should be set.
 *              If bit n (0-7) is set, LED n will be set.
 *              If bit n (0-7) is not set, LED n will be ignored.
 *  @param on   The new state of the LEDs.
 *              If bit n (0-7) is set (and bit n of mask is set), LED n will be switch on.
 *              If bit n (0-7) is not set (and bit n of mask is set), LED n will be switch off.
 */
void leds_set_mask(uint8_t mask, uint8_t on);

#endif /* LEDS_H_ */
