/* pcf8574 - Using the PCF8574 8-bit I/O expander for the I²C bus
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

#ifndef PCF8574_H_
#define PCF8574_H_

#include "openm128/types.h"
#include "openm128/generic_io.h"

/** The type of the PCF8574 device. */
typedef enum {
	kPCF8574  = 0,  ///< A PCF8574.
	kPCF8574A    ,  ///< A PCF8574A.

	kPCF8574MAX     ///< Invalid device, for range checks.
} pcf8574_type_t;

typedef struct {
	pcf8574_type_t type;
	uint8_t        device_id;
	generic_io_t   interrupt;
} pcf8574_t;

/** Initialize the PCF8574 device.
 *
 *  @param pcf8574   The PCF8574 structure to be filled with the device details.
 *  @param type      The type of the device.
 *  @param device_id The ID of the device.
 *  @param interrupt The pin used as an interrupt trigger.
 *
 *  @param FALSE if the type or device_id is out of range.
 */
bool pcf8574_init(pcf8574_t *pcf8574, pcf8574_type_t type, uint8_t device_id, generic_io_t interrupt);

/** Was an interrupt triggered at the interrupt pin? */
bool pcf8574_has_interrupt(const pcf8574_t *pcf8574);

/** Wait until an interrupt is triggered at the interrupt pin. */
bool pcf8574_wait_interrupt(const pcf8574_t *pcf8574);

/** Read the state of the IO pins. */
bool pcf8574_get(const pcf8574_t *pcf8574, uint8_t *data);

/** Set the state of the IO pins. */
bool pcf8574_set(pcf8574_t *pcf8574, uint8_t data);

#endif /* PCF8574_H_ */
