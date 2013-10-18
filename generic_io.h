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

/** @file generic_io.h
 *  These functions provide a method to abstract pin input/output. The hardware can then
 *  be rewired without the need to change the access code.
 *
 *  Of course, this comes with a price: The compiler loses the ability to optimize
 *  port access, and always uses the slow LD/ST instructions. Compared to direct port
 *  access, this is really slow. You have been warned.
 */

#ifndef GENERIC_IO_H_
#define GENERIC_IO_H_

#include "types.h"

typedef enum {
	kPortA,
	kPortB,
	kPortC,
	kPortD,
	kPortE,
	kPortF,
	kPortG
} port_t;

typedef struct {
	volatile uint8_t *reg_ddr;
	volatile uint8_t *reg_out;
	volatile uint8_t *reg_in;
	uint8_t pin_mask;
} generic_io_t;

/** Create a generic IO on that specific port and pin. */
generic_io_t generic_io_create(port_t port, uint8_t pin);

/** Make that generic IO an input. */
void generic_io_make_input(generic_io_t *gio, bool pullup);
/** Make that generic IO an output. */
void generic_io_make_output(generic_io_t *gio);

/** Read a value from the generic IO. */
bool generic_io_read(generic_io_t *gio);
/** Read up to 8 generic IOs and arrange them into a multi-bit value, LSB first and right-adjusted. */
uint8_t generic_io_read_multi(generic_io_t *gio, uint8_t count);

/** Set the bit on the generic IO. */
void generic_io_set(generic_io_t *gio);
/** Clear the bit on the generic IO. */
void generic_io_clear(generic_io_t *gio);
/** Write a value to the generic IO. */
void generic_io_write(generic_io_t *gio, bool value);
/** Write a right-adjusted multi-bit value to up to 8 generic IOs, LSB first. */
void generic_io_write_multi(generic_io_t *gio, uint8_t count, uint8_t value);

#endif /* GENERIC_IO_H_ */
