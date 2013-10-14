/* generic_io - Generically read from / write to pins of the ATmega128
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

#include "generic_io.h"

// Evil macro hackery to save me typing
#define GENERIC_IO_SET(g, d, o, i)  g.reg_ddr = &d; g.reg_out = &o; g.reg_in = &i;
#define GENERIC_IO_DEFINE(g, p) case kPort##p: GENERIC_IO_SET(g, DDR##p, PORT##p, PIN##p); break;
generic_io_t generic_io_create(port_t port, uint8_t pin) {
	generic_io_t gio;

	switch (port) {
		GENERIC_IO_DEFINE(gio, A);
		GENERIC_IO_DEFINE(gio, B);
		GENERIC_IO_DEFINE(gio, C);
		GENERIC_IO_DEFINE(gio, D);
		GENERIC_IO_DEFINE(gio, E);
		GENERIC_IO_DEFINE(gio, F);
		GENERIC_IO_DEFINE(gio, G);
	}

	gio.pin_mask = 1 << pin;

	return gio;
}

void generic_io_make_input(generic_io_t *gio, bool pullup) {
	*gio->reg_ddr &= ~gio->pin_mask;

	if (pullup)
		*gio->reg_out |=  gio->pin_mask;
	else
		*gio->reg_out &= ~gio->pin_mask;
}

void generic_io_make_output(generic_io_t *gio) {
	*gio->reg_ddr |= gio->pin_mask;
}

bool generic_io_read(generic_io_t *gio) {
	return !!(*gio->reg_in & gio->pin_mask);
}

uint8_t generic_io_read_multi(generic_io_t *gio, uint8_t count) {
	if (count > 8)
		return 0x00;

	uint8_t value = 0x00;

	for (uint8_t i = 0; i < count; i++)
		value |= generic_io_read(gio[i]) << (count - i - 1);

	return value;
}

void generic_io_set(generic_io_t *gio) {
	*gio->reg_out |= gio->pin_mask;
}

void generic_io_clear(generic_io_t *gio) {
	*gio->reg_out &= ~gio->pin_mask;
}

void generic_io_write(generic_io_t *gio, bool value) {
	if (value)
		generic_io_set(gio);
	else
		generic_io_clear(gio);
}

void generic_io_write_multi(generic_io_t *gio, uint8_t count, uint8_t value) {
	if (count > 8)
		return;

	for (uint8_t i = 0; i < count; i++)
		generic_io_write(gio[i], value & (1 << (count - i - 1)));
}