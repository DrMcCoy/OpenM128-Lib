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

#include "openm128/types.h"
#include "openm128/lcdfont.h"

// Dimensions

#define LCD22_WIDTH  240 // LCD screen width in pixels
#define LCD22_HEIGHT 320 // LCD screen height in pixels

// Colors, RGB565 (5 bit red, 6 bit green, 5 bit blue)

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

#define LCD22_COLOR(R, G, B) (uint16_t)(((((R) >> 3) & 0x1F) << 11) | ((((G) >> 2) & 0x3F) << 5) | ((((B) >> 3) & 0x1F) << 0))

#define LCD22_COLOR_RED(color)   (uint8_t)((((color) >> 11) & 0x1F) << 3)
#define LCD22_COLOR_GREEN(color) (uint8_t)((((color) >>  5) & 0x3F) << 2)
#define LCD22_COLOR_BLUE(color)  (uint8_t)((((color) >>  0) & 0x1F) << 3)

/** Initialize the 2.2" color display. */
void lcd22_init(void);

/** Clear the whole screen of the display with a solid color. */
void lcd22_clear_screen(uint16_t color);
/** Clear a rectangular area of the display with a solid color. */
void lcd22_clear_area(int16_t x, int16_t y, int16_t width, int16_t height, uint16_t color);

/** Draw a dot onto the display. */
void lcd22_draw_dot(int16_t x, int16_t y, int16_t size, uint16_t color);

/** Draw a line onto the display. */
void lcd22_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t thickness, uint16_t color);

/** Draw a rectangle onto the display. */
void lcd22_draw_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t thickness, uint16_t color);
/** Draw a filled rectangle onto the display. */
void lcd22_draw_filled_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
/** Draw a rectangle filled with a horizontal color gradient onto the display. */
void lcd22_draw_hgradient_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color1, uint16_t color2);
/** Draw a rectangle filled with a vertical color gradient onto the display. */
void lcd22_draw_vgradient_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color1, uint16_t color2);

/** Draw a circle onto the display. */
void lcd22_draw_circle(int16_t x0, int16_t y0, int16_t radius, int16_t thickness, uint16_t color);
/** Draw a filled circle onto the display. */
void lcd22_draw_filled_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color);
/** Draw a circle filled with a horizontal color gradient onto the display. */
void lcd22_draw_hgradient_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color1, uint16_t color2);
/** Draw a circle filled with a vertical color gradient onto the display. */
void lcd22_draw_vgradient_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color1, uint16_t color2);

/* Note: The following functions to *not* clip smartly. They only make sure that nothing "outside" of the
 *       visible screen range is overwritten. If part of the destination area lies outside the screen, you
 *       will *not* get the part that would be visible, you will get broken images / characters.
 */

/** Draw a 7-bit ASCII character in the specified font on the display. */
void lcd22_draw_char(char c, int16_t x, int16_t y,
                     uint16_t foreground_color, uint16_t background_color, lcdfont_t font);
/** Draw a string in the specified font onto the display. */
void lcd22_draw_string(const char *str, int16_t x, int16_t y,
                       uint16_t foreground_color, uint16_t background_color, lcdfont_t font);
/** Draw a string held in program memory in the specified font onto the display. */
void lcd22_draw_string_P(const char *str, int16_t x, int16_t y,
                         uint16_t foreground_color, uint16_t background_color, lcdfont_t font);

/* Note: The following functions exist in up to 3 versions:
 * - Functions that expect both bitmap and palette in RAM
 * - Functions that expect the bitmap in program memory and the palette in RAM (_P)
 * - Functions that expect both the bitmap and the palette in program memory (_PP)
 */

/** Draw a monochrome (1 bit per pixel) bitmap onto the display.
 *
 *  @param bitmap Image data, one bit per pixel. The most significant bit is the left-most pixel.
 *  @param x      x coordinate of the destination position.
 *  @param y      y coordinate of the destination position.
 *  @param width  Width of the bitmap data.
 *                Note: If the width is not divisible by 8, each row of the bitmap data needs to be padded to a full byte.
 *  @param height Height of the bitmap data.
 *  @param foreground_color color to use for set (1) bits in the bitmap data.
 *  @param background_color color to use for unset (0) bits in the bitmap data.
 */
void lcd22_draw_bitmap_1bpp(const uint8_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height,
                            uint16_t foreground_color, uint16_t background_color);

/** Draw a monochrome (1 bit per pixel) bitmap held in program space onto the display. */
void lcd22_draw_bitmap_1bpp_P(const uint8_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height,
                              uint16_t foreground_color, uint16_t background_color);

/** Draw a monochrome (1 bit per pixel) bitmap held in program space onto the display. */
void lcd22_draw_bitmap_1bpp_PP(const uint8_t *bitmap, const uint16_t *palette,
                               int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 16bpp color bitmap onto the display. */
void lcd22_draw_bitmap_16bpp(const uint16_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 16bpp color bitmap help in program space onto the display. */
void lcd22_draw_bitmap_16bpp_P(const uint16_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 24bpp RGB color bitmap onto the display.
 *
 *  Note: The bitmap will be converted to RGB565 16bpp data on-the-fly.
 */
void lcd22_draw_bitmap_24bpp(const uint8_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 24bpp RGB color bitmap held in program space onto the display. */
void lcd22_draw_bitmap_24bpp_P(const uint8_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 2bpp palette'd bitmap onto the display. */
void lcd22_draw_bitmap_2bpp(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 2bpp palette'd bitmap held in program space onto the display. */
void lcd22_draw_bitmap_2bpp_P(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 2bpp palette'd bitmap held in program space onto the display. */
void lcd22_draw_bitmap_2bpp_PP(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 4bpp palette'd bitmap onto the display. */
void lcd22_draw_bitmap_4bpp(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 4bpp palette'd bitmap held in program space onto the display. */
void lcd22_draw_bitmap_4bpp_P(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 4bpp palette'd bitmap held in program space onto the display. */
void lcd22_draw_bitmap_4bpp_PP(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 8bpp palette'd bitmap onto the display. */
void lcd22_draw_bitmap_8bpp(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 8bpp palette'd bitmap held in program space onto the display. */
void lcd22_draw_bitmap_8bpp_P(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height);

/** Draw a 8bpp palette'd bitmap held in program space onto the display. */
void lcd22_draw_bitmap_8bpp_PP(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height);

/** Query the state of the touch screen.
 *
 *  @param x The x coordinate of the touch position.
 *  @param y The y coordinate of the touch position.
 *
 *  @return TRUE if the touch screen is currently pressed down, FALSE otherwise.
 */
bool lcd22_get_touch(int16_t *x, int16_t *y);

#endif /* LCD22_H_ */
