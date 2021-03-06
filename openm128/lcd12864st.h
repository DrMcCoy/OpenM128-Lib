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

/** @file lcd12864st.h
 *  The character encoding used in the display is EUC-CN. Each memory "cell" consists of 2 bytes,
 *  and each cell can either contain two 8x16 7-bit ASCII characters (if the bytes are within
 *  the range 0-127), or one 16x16 GB 2312 character.
 *
 *  For details on that character encoding, see <https://en.wikipedia.org/wiki/GB_2312> and
 *  <http://www.khngai.com/chinese/charmap/tblgb.php>.
 *
 *  Please also note that display does not like other devices being on the same SPI bus.
 *  Specifically, the LCD controller resets itself when CS is activated and it doesn't
 *  like the clock or data line being toggled while CS is inactive.
 */

#ifndef LCD12864ST_H_
#define LCD12864ST_H_

#include <stdarg.h>

#include "openm128/types.h"

#define LCD12864ST_LINES    4
#define LCD12864ST_COLUMNS 16

/** Initialize the display. */
void lcd12864st_init(void);

/** Clear the whole display. */
void lcd12864st_clear(void);
/** Clear one line (0-3) of the display. */
void lcd12864st_clearLine(uint8_t line);

/** Fill the whole display with a character. */
void lcd12864st_fill(char c);
/** Fill a line of the display with a character. */
void lcd12864st_fillLine(uint8_t line, char c);

/** Print a string onto line y (0-3) of the display, starting with column x (0-15). */
void lcd12864st_print(uint8_t x, uint8_t y, const char *str);
/** Print a string held in program memory onto line y (0-3) of the display, starting with column x (0-15). */
void lcd12864st_print_P(uint8_t x, uint8_t y, const char *str);

/** Print a string onto the display and center it onto the specified line. */
void lcd12864st_print_centered(uint8_t line, const char *str);

/** Print a string help in memory onto the display and center it onto the specified line. */
void lcd12864st_print_centered_P(uint8_t line, const char *str);

/** Print a formated string onto the display. See printf() for details. */
void lcd12864st_printf(uint8_t x, uint8_t y, const char *format, ...);

/** Refresh the whole display. */
void lcd12864st_refresh(void);
/** Refresh just one line. */
void lcd12864st_refreshLine(uint8_t line);

#endif /* LCD12864ST_H_ */
