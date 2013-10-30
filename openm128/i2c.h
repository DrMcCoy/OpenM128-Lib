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
 *
 *  The slave address used here is the 7 bit identifying the slave device, i.e. not
 *  including the R/W bit, right-aligned. For example, if the data sheet of a device
 *  mentions that the control byte for writing is 10100000 (0xA0) and for reading is
 *  10100001 (0xA1), then the slave address is 010100000 (0x50), or 0xA0 >> 1.
 */

#ifndef I2C_H_
#define I2C_H_

#include "openm128/types.h"

/** An operation to poll for. */
typedef enum {
	kI2CPollRead,
	kI2CPollWrite,
	kI2CPollReadWrite
} i2c_poll_t;


/** Initialize the I²C in master mode. */
void i2c_init(void);

/** Poll a slave.
 *
 *  @param slave_address The address of the slave to poll.
 *  @param type          The type of the operation to poll for.
 *
 *  @return TRUE if the slave acknowledged the request.
 */
bool i2c_poll(uint16_t slave_address, i2c_poll_t type);

/** Send a command to a slave.
 *
 *  @param slave_address The address of the slave to send the command to.
 *  @param command       The command to send.
 *
 *  @return TRUE if the command was successfully sent.
 */
bool i2c_send_command(uint16_t slave_address, uint8_t command);

/** Write n bytes of data directly to a slave.
 *
 *  @param slave_address The address of the slave to write to.
 *  @param n             The number of bytes to write.
 *  @param data          The data to write.
 *
 *  @return The number of bytes successfully written.
 */
uint16_t i2c_write(uint16_t slave_address, uint16_t n, const uint8_t *data);

/** Read n bytes of data directly from a slave.
 *
 *  @param slave_address The address of the slave to read from.
 *  @param n             The number of bytes to read.
 *  @param data          The read data will be stored here.
 *
 *  @return The number of bytes successfully read.
 */
uint16_t i2c_read(uint16_t slave_address, uint16_t n, uint8_t *data);

/** Write one bytes of data directly to a slave.
 *
 *  @param slave_address The address of the slave to write to.
 *  @param data          The data to write.
 *
 *  @return TRUE if the byte was successfully written.
 */
bool i2c_write_byte(uint16_t slave_address, uint8_t data);

/** Write n bytes of data to an 8-bit memory address of a slave.
 *
 *  @param slave_address The address of the slave to write to.
 *  @param data_address  The memory address within the slave to write to.
 *  @param n             The number of bytes to write.
 *  @param data          The data to write.
 *
 *  @return The number of bytes successfully written.
 */
uint16_t i2c_write8(uint16_t slave_address, uint8_t data_address, uint16_t n, const uint8_t *data);

/** Read n bytes of data from an 8-bit memory address of a slave.
 *
 *  @param slave_address The address of the slave to read from.
 *  @param data_address  The memory address within the slave to read from.
 *  @param n             The number of bytes to read.
 *  @param data          The read data will be stored here.
 *
 *  @return The number of bytes successfully read.
 */
uint16_t i2c_read8(uint16_t slave_address, uint8_t data_address, uint16_t n, uint8_t *data);

/** Write one bytes of data to an 8-bit memory address of a slave.
 *
 *  @param slave_address The address of the slave to write to.
 *  @param data_address  The memory address within the slave to write to.
 *  @param data          The data to write.
 *
 *  @return TRUE if the byte was successfully written.
 */
bool i2c_write8_byte(uint16_t slave_address, uint8_t data_address, uint8_t data);

/** Write n bytes of data to a 16-bit memory address of a slave.
 *
 *  @param slave_address The address of the slave to write to.
 *  @param data_address  The memory address within the slave to write to.
 *  @param n             The number of bytes to write.
 *  @param data          The data to write.
 *
 *  @return The number of bytes successfully written.
 */
uint16_t i2c_write16(uint16_t slave_address, uint16_t data_address, uint16_t n, const uint8_t *data);

/** Read n bytes of data from a 16-bit memory address of a slave.
 *
 *  @param slave_address The address of the slave to read from.
 *  @param data_address  The memory address within the slave to read from.
 *  @param n             The number of bytes to read.
 *  @param data          The read data will be stored here.
 *
 *  @return The number of bytes successfully read.
 */
uint16_t i2c_read16(uint16_t slave_address, uint16_t data_address, uint16_t n, uint8_t *data);

/** Write one bytes of data to a 16-bit memory address of a slave.
 *
 *  @param slave_address The address of the slave to write to.
 *  @param data_address  The memory address within the slave to write to.
 *  @param data          The data to write.
 *
 *  @return TRUE if the byte was successfully written.
 */
bool i2c_write16_byte(uint16_t slave_address, uint16_t data_address, uint8_t data);

#endif /* I2C_H_ */
