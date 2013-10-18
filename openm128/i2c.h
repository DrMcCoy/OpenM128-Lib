/* i2c - Communication using the I²C interface
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

/** @file i2c.h
 *  For some reason, avr-gcc 4.5.1 breaks I²C communication with compiling with -O3.
 *  If you get weird errors, try reducing optimization to -O2.
 */

#ifndef I2C_H_
#define I2C_H_

#include "openm128/types.h"

/** Initialize the I²C in master mode. */
void i2c_init();

/** Write n bytes of data to a memory address of a slave. */
bool i2c_write(uint16_t slave_address, uint8_t data_address, uint16_t n, const uint8_t *data);
/** Read n bytes of data from a memory address of a slave. */
bool i2c_read(uint16_t slave_address, uint8_t data_address, uint16_t n, uint8_t *data);

/** Write one byte to a memory address of a slave. */
bool i2c_write_byte(uint16_t slave_address, uint8_t data_address, uint8_t data);

#endif /* I2C_H_ */
