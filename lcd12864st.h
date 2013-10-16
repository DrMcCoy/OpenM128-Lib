/* lcd12864st - Waveshare LCD12864ST functions
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

#ifndef LCD12864ST_H_
#define LCD12864ST_H_

#include <stdarg.h>

#include "types.h"

#define LCD12864ST_LINES    4
#define LCD12864ST_COLUMNS 16

/** Initialize the display. */
void lcd12864st_init();

/** Clear the whole display. */
void lcd12864st_clear();
/** Clear one line (0-3) of the display. */
void lcd12864st_clearLine(uint8_t line);

/** Print a string onto a line (0-3) of the display, starting with the specified column (0-15). */
void lcd12864st_print(uint8_t line, uint8_t column, const char *str);
/** Print a string held in program memory onto a line (0-3) of the display, starting with the specified column (0-15). */
void lcd12864st_print_P(uint8_t line, uint8_t column, const char *str);

/** Print a formated string onto the display. See printf() for details. */
void lcd12864st_printf(uint8_t line, uint8_t column, const char *format, ...);

/** Refresh the whole display. */
void lcd12864st_refresh();
/** Refresh just one line. */
void lcd12864st_refreshLine(uint8_t line);

#endif /* LCD12864ST_H_ */