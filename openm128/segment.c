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

#include "openm128/segment.h"

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
                                      generic_io_t dot) {

	segment->ct[0] = ct1;
	segment->ct[1] = ct2;
	segment->ct[2] = ct3;
	segment->ct[3] = ct4;

	segment->pt = pt;

	segment->seg[0] = a;
	segment->seg[1] = b;
	segment->seg[2] = c;
	segment->seg[3] = d;
	segment->seg[4] = e;
	segment->seg[5] = f;
	segment->seg[6] = g;
	segment->seg[7] = dot;

	generic_io_make_output(&segment->ct[0]);
	generic_io_make_output(&segment->ct[1]);
	generic_io_make_output(&segment->ct[2]);
	generic_io_make_output(&segment->ct[3]);

	generic_io_make_output(&segment->pt);

	generic_io_make_output(&segment->seg[0]);
	generic_io_make_output(&segment->seg[1]);
	generic_io_make_output(&segment->seg[2]);
	generic_io_make_output(&segment->seg[3]);
	generic_io_make_output(&segment->seg[4]);
	generic_io_make_output(&segment->seg[5]);
	generic_io_make_output(&segment->seg[6]);
	generic_io_make_output(&segment->seg[7]);
}

void segment_set_state(segment_t *segment, uint8_t digit, uint8_t value) {
	generic_io_write_multi(segment->seg, 8, 0xFF);

	generic_io_write_multi(segment->ct , 4, 1 << digit);
	generic_io_write_multi(segment->seg, 8, ~value);
}

static uint8_t hex_chars[16] = {
	0x3F, 0x06, 0x5B, 0x4F,
	0x66, 0x6D, 0x7D, 0x07,
	0x7F, 0x6F, 0x77, 0x7C,
	0x39, 0x5E, 0x79, 0x71
};

void segment_set_hex(segment_t *segment, uint8_t digit, uint8_t hex) {
	segment_set_state(segment, digit, hex_chars[hex % 16]);
}

void segment_set_colon(segment_t *segment, bool on) {
	generic_io_write(&segment->pt, !on);
}

void segment_set_full(segment_t *segment, uint8_t value1, uint8_t value2, uint8_t value3, uint8_t value4, bool colon) {
	segment_set_state(segment, 0, value1);
	segment_set_state(segment, 1, value2);
	segment_set_state(segment, 2, value3);
	segment_set_state(segment, 3, value4);
	segment_set_colon(segment, colon);
}

void segment_set_full_hex(segment_t *segment, uint8_t hex1, uint8_t hex2, uint8_t hex3, uint8_t hex4, bool colon) {
	segment_set_hex(segment, 0, hex1);
	segment_set_hex(segment, 1, hex2);
	segment_set_hex(segment, 2, hex3);
	segment_set_hex(segment, 3, hex4);
	segment_set_colon(segment, colon);
}
