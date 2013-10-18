/* segment - Display information using the 8 segment LED display
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

/** @file segment.h
 *  Note: The display module is quite "stupid": to display 4 different digits, you have
 *        to constantly refresh them all. Otherwise, all enabled digits will show the same.
 */

#ifndef SEGMENT_H_
#define SEGMENT_H_

#include "openm128/types.h"
#include "openm128/generic_io.h"

typedef struct {
	generic_io_t ct[4];
	generic_io_t pt;
	generic_io_t seg[8];
} segment_t;

void segment_init(segment_t *segment, generic_io_t ct1,
                                      generic_io_t ct2,
                                      generic_io_t ct3,
                                      generic_io_t ct4,
                                      generic_io_t pt,
                                      generic_io_t a,
                                      generic_io_t b,
                                      generic_io_t c,
                                      generic_io_t d,
                                      generic_io_t e,
                                      generic_io_t f,
                                      generic_io_t g,
                                      generic_io_t dot);

/** Set the state of a digit.
 *
 *  Each bit in value represents one segment in the digit.
 */
void segment_set_state(segment_t *segment, uint8_t digit, uint8_t value);

/** Set the state of a digit representing a hexadecimal character.
 *
 *  Switch on segments to show a hexadecimal character (0 - F).
 */
void segment_set_hex(segment_t *segment, uint8_t digit, uint8_t hex);

/** Set the state of the colon segment. */
void segment_set_colon(segment_t *segment, bool on);

/** Set the state of all digits, left to right. */
void segment_set_full(segment_t *segment, uint8_t value1, uint8_t value2, uint8_t value3, uint8_t value4, bool colon);

/** Set the state of all digits, left to right, with hex characters. */
void segment_set_full_hex(segment_t *segment, uint8_t hex1, uint8_t hex2, uint8_t hex3, uint8_t hex4, bool colon);

#endif /* SEGMENT_H_ */
