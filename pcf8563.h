/* pcf8563 - Reading the PCF8563 real-time clock (RTC) using I²C
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

#ifndef PCF8563_H_
#define PCF8563_H_

#include "types.h"

/** A timestamp returned by the PCF8563. */
typedef struct {
	bool integrity; ///< TRUE if the clock's integrity is guaranteed.

	uint8_t second; ///< (0-59).
	uint8_t minute; ///< (0-59).
	uint8_t hour;   ///< (0-23).

	uint8_t  day;      ///< (1-31).
	uint8_t  weekday;  ///< (0-6).
	uint8_t  month;    ///< (1-12).
	uint16_t year;     ///< (1900-2099).
} pcf8563_time_t;

/** Initialize the PCF8563 RTC. */
bool pcf8563_init();

/** Try to initialize the PCF8563 until it works. */
void pcf8563_init_wait();

/** Does the PCF8563 currently have a valid timestamp? */
bool pcf8563_has_valid_time();

/** Get the current time from the PCF8563. */
bool pcf8563_get(pcf8563_time_t *time);

/** Set the current time. */
bool pcf8563_set(pcf8563_time_t time);

/** Reset the PCF8563 to "time 0" (1900-01-01T00:00:00). */
bool pcf8563_reset();

/** Check if a certain timestamp is valid (within the range supported by the PCF8563). */
bool pcf8563_time_valid(const pcf8563_time_t *time);

/** Return a timestamp representing "time 0" (1900-01-01T00:00:00). */
pcf8563_time_t pcf8563_time_0();
/** Return an invalid timestamp. */
pcf8563_time_t pcf8563_time_invalid();
/** Return a timestamp corresponding to the compile time. */
pcf8563_time_t pcf8563_time_compile();

/** Calculate the day of the week from day (of the month), month and year. */
uint8_t pcf8563_calculate_day_of_week(uint8_t day, uint8_t month, uint16_t year);

#endif /* PCF8563_H_ */
