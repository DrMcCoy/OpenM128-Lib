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

#include <stdio.h>
#include <string.h>

#include <util/delay.h>

#include "openm128/pcf8563.h"
#include "openm128/i2c.h"

#define fromBCD(x) ( (((x) >> 4) * 10 ) + ((x) % 16) )
#define toBCD(x)   ( (((x) / 10) << 4 ) + ((x) % 10) )

#define PCF8563_SLAVE_ADDRESS  (0xA2 >> 1)

#define PCF8563_SLAVE_ADDRESS_WRITE   (PCF8563_SLAVE_ADDRESS << 1)
#define PCF8563_SLAVE_ADDRESS_READ   ((PCF8563_SLAVE_ADDRESS << 1) | 1)

#define PCF8563_ADDRESS_CTRL1  0x00
#define PCF8563_ADDRESS_CTRL2  0x01

#define PCF8563_ADDRESS_SECOND  0x02
#define PCF8563_ADDRESS_MINUTE  0x03
#define PCF8563_ADDRESS_HOUR    0x04

#define PCF8563_ADDRESS_DAY    0x05
#define PCF8563_ADDRESS_WEEK   0x06
#define PCF8563_ADDRESS_MONTH  0x07
#define PCF8563_ADDRESS_YEAR   0x08

bool pcf8563_init(void) {
	i2c_init();

	// Normal operation
	if (!i2c_write8_byte(PCF8563_SLAVE_ADDRESS, PCF8563_ADDRESS_CTRL1, 0x00))
		return FALSE;

	// Disable alarm
	if (!i2c_write8_byte(PCF8563_SLAVE_ADDRESS, PCF8563_ADDRESS_CTRL1, 0x00))
		return FALSE;

	return TRUE;
}

void pcf8563_init_wait(void) {
	while (!pcf8563_init());
}

bool pcf8563_has_valid_time(void) {
	uint8_t data;
	if (i2c_read8(PCF8563_SLAVE_ADDRESS, PCF8563_ADDRESS_SECOND, 1, &data) != 1)
		return FALSE;

	return !(data & 0x80);
}

bool pcf8563_get(pcf8563_time_t *time) {
	uint8_t data[7];
	if (i2c_read8(PCF8563_SLAVE_ADDRESS, PCF8563_ADDRESS_SECOND, 7, data) != 7)
		return FALSE;

	time->integrity = !(data[0] & 0x80);

	time->second = fromBCD(data[0] & 0x7F);
	time->minute = fromBCD(data[1] & 0x7F);
	time->hour   = fromBCD(data[2] & 0x3F);

	time->day     = fromBCD(data[3] & 0x3F);
	time->weekday = fromBCD(data[4] & 0x07);
	time->month   = fromBCD(data[5] & 0x1F);
	time->year    = fromBCD(data[6] & 0xFF) + ((data[5] & 0x80) ? 2000 : 1900);

	return TRUE;
}

bool pcf8563_set(pcf8563_time_t time) {
	if (!pcf8563_time_valid(&time))
		return FALSE;

	uint8_t data[7];

	data[0] = toBCD(time.second) | (time.integrity ? 0x00 : 0x80);
	data[1] = toBCD(time.minute);
	data[2] = toBCD(time.hour);

	data[3] = toBCD(time.day);
	data[4] = toBCD(time.weekday);
	data[5] = toBCD(time.month) | ((time.year > 1999) ? 0x80 : 0x00);
	data[6] = toBCD(time.year % 100);

	return i2c_write8(PCF8563_SLAVE_ADDRESS, PCF8563_ADDRESS_SECOND, 7, data) == 7;
}

bool pcf8563_reset(void) {
	return pcf8563_set(pcf8563_time_0());
}

bool pcf8563_time_valid(const pcf8563_time_t *time) {
	if ((time->second > 59) || (time->minute > 59) || (time->hour > 23))
		return FALSE;

	if ((time->day < 1) || (time->day > 31) || (time->weekday > 6))
		return FALSE;
	if ((time->month < 1) || (time->month > 12))
		return FALSE;
	if ((time->year < 1900) || (time->year > 2099))
		return FALSE;

	return TRUE;
}

pcf8563_time_t pcf8563_time_0(void) {
	pcf8563_time_t time;

	time.integrity = TRUE;

	time.second = 0;
	time.minute = 0;
	time.hour   = 0;

	time.day     = 1;
	time.weekday = 0;
	time.month   = 1;
	time.year    = 1900;

	return time;
}

pcf8563_time_t pcf8563_time_invalid(void) {
	pcf8563_time_t time;

	time.integrity = FALSE;

	time.second = 0xFF;
	time.minute = 0xFF;
	time.hour   = 0xFF;

	time.day     = 0xFF;
	time.weekday = 0xFF;
	time.month   = 0xFF;
	time.year    = 0xFFFF;

	return time;
}

pcf8563_time_t pcf8563_time_compile(void) {
	static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

	char month_string[4];
	int hour, minute, second;
	int year, month, day;

	sscanf(__DATE__, "%3s %2d %4d", month_string, &day, &year);
	sscanf(__TIME__, "%02d:%02d:%02d", &hour, &minute, &second);

	month = ((strstr(month_names, month_string) - month_names) / 3) + 1;

	pcf8563_time_t time;

	time.integrity = TRUE;

	time.second = second;
	time.minute = minute;
	time.hour   = hour;

	time.day   = day;
	time.month = month;
	time.year  = year;

	time.weekday = pcf8563_calculate_day_of_week(day, month, year);

	return time;
}

uint32_t pcf8563_time_to_unix(const pcf8563_time_t *time) {
	if (!pcf8563_time_valid(time) || (time->year < 1970))
		return 0xFFFFFFFF;

	const uint16_t days_to_month_start[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

	const uint32_t years      = time->year - 1970;
	const uint32_t leap_years = ((time->year - 1) - 1968) / 4 - ((time->year - 1) - 1900) / 100 + ((time->year - 1) - 1600) / 400;

	uint32_t unix = 0;
	unix += time->second + 60L * time->minute + 60L * 60L * time->hour;
	unix += (days_to_month_start[time->month - 1] + time->day - 1L) * 60L * 60L * 24L;
	unix += (years * 365L + leap_years) * 60L * 60L * 24L;

	if ((time->month > 2) && (time->year %4 == 0 && (time->year % 100 != 0 || time->year % 400 == 0)))
		unix += 60L * 60L * 24L;

	return unix;
}

void pcf8563_time_from_unix(pcf8563_time_t *time, uint32_t unix) {
	uint32_t day = unix / (24L * 60L * 60L);
	uint32_t secs = unix % (24L * 60L * 60L);
	uint32_t mins = secs / 60;

	time->hour = mins / 60;
	time->minute = mins % 60;
	time->second = secs % 60;

	uint32_t year = (((day * 4) + 2) / 1461);
	bool leap = (time->year %4 == 0 && (time->year % 100 != 0 || time->year % 400 == 0));

	time->year = year + 1970;

	day -= ((year * 1461) + 1) / 4;

	time->day = day;

	day += (day > (uint32_t)(58 + leap)) ? ((leap) ? 1 : 2) : 0;

	time->month = ((day * 12) + 6) / 367;
	time->day = day + 1 - ((time->month * 367) + 5) / 12;

	time->month++;
	time->weekday = pcf8563_calculate_day_of_week(time->day, time->month, time->year);
	time->integrity = TRUE;
}

/** Claus Tøndering's algorithm.
 *  See https://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week#T.C3.B8ndering.27s_algorithm
 *  for specifics.
 */
uint8_t pcf8563_calculate_day_of_week(uint8_t day, uint8_t month, uint16_t year) {
	static uint8_t t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

	year -= month < 3;

	return ((year + year/4 - year/100 + year/400 + t[month-1] + day) + 6) % 7;
}
