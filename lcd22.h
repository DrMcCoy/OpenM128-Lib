/* lcd22 - Waveshare 2.2" color display functions
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

#ifndef LCD22_H_
#define LCD22_H_

#include "types.h"

// Dimensions

#define LCD22_WIDTH  240 // LCD screen width in pixels
#define LCD22_HEIGHT 320 // LCD screen height in pixels

#define LCD22_CHAR_WIDTH   8 // Width of a character in pixels
#define LCD22_CHAR_HEIGHT 16 // Height of a character in pixels

#define LCD22_NUM_CHAR_X (LCD22_WIDTH  / LCD22_CHAR_WIDTH ) // Number of characters that fit onto the screen horizontally
#define LCD22_NUM_CHAR_Y (LCD22_HEIGHT / LCD22_CHAR_HEIGHT) // Number of characters that fit onto the screen vertically

// Colors

#define  LCD22_CYAN       0x07FF
#define  LCD22_PURPLE     0xF81F
#define  LCD22_RED        0xF800
#define  LCD22_GREEN      0x07E0
#define  LCD22_BLUE       0x001F
#define  LCD22_WHITE      0xFFFF
#define  LCD22_BLACK      0x0000
#define  LCD22_YELLOW     0xFFE0
#define  LCD22_ORANGE     0xFC08
#define  LCD22_GRAY       0x8430
#define  LCD22_LGRAY      0xC618
#define  LCD22_DARKGRAY   0x8410
#define  LCD22_PORPO      0x801F
#define  LCD22_PINK       0xF81F
#define  LCD22_GRAYBLUE   0x5458
#define  LCD22_LGRAYBLUE  0xA651
#define  LCD22_DARKBLUE   0x01CF
#define  LCD22_LIGHTBLUE  0x7D7C


/** Initialize the 2.2" color display. */
void lcd22_init();

/** Clear the whole screen of the display with a solid color. */
void lcd22_clear_screen(uint16_t color);
/** Clear a rectangular area of the display with a solid color. */
void lcd22_clear_area(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

/** Draw an ASCII character on the display. */
void lcd22_draw_char(char c, uint16_t x, uint16_t y, uint16_t foreground_color, uint16_t background_color);
/** Draw a string onto the display. */
void lcd22_draw_string(const char *str, uint16_t x, uint16_t y, uint16_t foreground_color, uint16_t background_color);

/** Draw a dot onto the display. */
void lcd22_draw_dot(int16_t x, int16_t y, uint16_t color);
/** Draw a line onto the display. */
void lcd22_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
/** Draw a circle onto the display. */
void lcd22_draw_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color);

#endif /* LCD22_H_ */