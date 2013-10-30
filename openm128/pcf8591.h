/* pcf8591 - Using the PCF8591 A/D and D/A converter for the I²C bus
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

#ifndef PCF8591_H_
#define PCF8591_H_

#include "openm128/types.h"

/** Input modes of the PCF8591. */
typedef enum {
	kPCF8591InputSingle     = 0, ///< 4 single-ended inputs (pins 0, 1, 2, 3).
	kPCF8591InputDiff3      = 1, ///< 3 differential inputs (pins 0+3, 1+3, 2+3).
	kPCF8591InputDiffSingle = 2, ///< Mixed input: 2 single-ended inputs (pins 0, 1) and one differential input (pins 2+3).
	kPCF8591InputDiff2      = 3, ///< 2 differential inputs (pins 0+1, 2+3).

	kPCF8591InputMAX ///< Invalid mode, for range checks.
} pcf8591_input_mode_t;

/** Initialize a PCF8591 ADC/DAC device. */
bool pcf8591_init(uint8_t device_id);

/** Convert the voltage on the specified channel of a PCF8591 device into a digital value. */
bool pcf8591_get(uint8_t device_id, pcf8591_input_mode_t mode, uint8_t channel, uint8_t *data);

/** Convert the voltage off all channels of a PCF8591 device into a digital value. */
bool pcf8591_get_all(uint8_t device_id, pcf8591_input_mode_t mode, uint8_t *data);

/** Set the voltage of the output pin of a PCF8591 device. */
bool pcf8591_set(uint8_t device_id, uint8_t data);

#endif /* PCF8591_H_ */
