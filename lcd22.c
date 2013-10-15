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

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "lcd22.h"

#include "util.h"

// -- Utility macros --

#define LCD22_RST_H()        PORTB |=  (1<<PB6)
#define LCD22_RST_L()        PORTB &= ~(1<<PB6)

#define LCD22_RS_H()         PORTB |=  (1<<PB5)
#define LCD22_RS_L()         PORTB &= ~(1<<PB5)

#define LCD22_CS_H()         PORTD |=  (1<<PD6)
#define LCD22_CS_L()         PORTD &= ~(1<<PD6)

#define LCD22_TOUCH_CS_H()   PORTB |=  (1<<PB4)
#define LCD22_TOUCH_CS_L()   PORTB &= ~(1<<PB4)

// -- Internal LCD support functions --

// Initialize the SPI I/O
static void lcd22_spio_init(void) {
	// Set direction of LCD pins
	DDRB = (DDRB & ~0x7F) | 0x77;
	DDRD |= 0x40;
	DDRE &= ~0x10;

	// Clear LCD pins / pull-up register setting
	PORTB &= ~0x7F;
	PORTD &= ~0x40;
	PORTE &= ~0x10;

	// Deactivate touchscreen CS
	LCD22_TOUCH_CS_H();

	// Deactivate LCD CS
	LCD22_RS_H();
	LCD22_CS_H();

	// Setup SPI
	SPCR = 0x50; // Enable SPI + SPI master (no interrupt, MSB first, leading rising edge, fosc/4 clock)
	SPSR = 0x00; // Clear SPI status register
}

// Send data over the SPI
static uint8_t lcd22_spio_send(uint8_t c) {
	SPDR = c;
	while (!(SPSR & (1 << SPIF))); // Wait for transmission to finish
	return SPDR;
}

// Select a register on the LCD controller
static void lcd22_write_reg(uint16_t index) {
	LCD22_RS_L();
	LCD22_CS_L();
	lcd22_spio_send((uint8_t)(index >> 8));
	lcd22_spio_send((uint8_t)(index));
	LCD22_CS_H();
	LCD22_RS_H();
}

// Send a data word to the LCD controller
static void lcd22_write_data(uint16_t data) {
	lcd22_spio_send((uint8_t)(data >> 8));
	lcd22_spio_send((uint8_t)(data));
}

// Send a command (register + data) to the LCD controller
static void lcd22_write_command(uint16_t index, uint16_t data) {
	lcd22_write_reg(index);

	LCD22_CS_L();
	lcd22_write_data(data);
	LCD22_CS_H();
}

// Prepare for mass data write to the LCD controller
static void lcd22_prepare_write() {
	// RAM write index (prepare write)
	LCD22_RS_L();
	lcd22_write_reg(0x202);
	LCD22_CS_L();
}

// Finish up mass data write to the LCD controller
static void lcd22_finish_write() {
	LCD22_RS_L();
	LCD22_CS_H();
}

// Set a draw area on the LCD screen
static bool lcd22_set_draw_area(int16_t x, int16_t y, int16_t width, int16_t height,
                                int16_t *left, int16_t *top, int16_t *right, int16_t *bottom) {

	if ((width <= 0) || (height <= 0))
		// Nothing to do
		return FALSE;

	*right  = x + width  - 1;
	*bottom = y + height - 1;

	if (((x < 0) && (*right  < 0)) || ((x >= LCD22_WIDTH ) && (*right  >= LCD22_WIDTH )) ||
	    ((y < 0) && (*bottom < 0)) || ((y >= LCD22_HEIGHT) && (*bottom >= LCD22_HEIGHT)))
		// Area lies wholly outside the screen
		return FALSE;

	// Clamp to valid area
	*left   = CLIP(x      , 0, LCD22_WIDTH  - 1);
	*top    = CLIP(y      , 0, LCD22_HEIGHT - 1);
	*right  = CLIP(*right , 0, LCD22_WIDTH  - 1);
	*bottom = CLIP(*bottom, 0, LCD22_HEIGHT - 1);

	if ((*left > *right) || (*top > *bottom))
		// Nothing to do
		return FALSE;

	// Set draw window
	lcd22_write_command(0x210, *left);
	lcd22_write_command(0x212, *top);
	lcd22_write_command(0x211, *right);
	lcd22_write_command(0x213, *bottom);

	// RAM position
	lcd22_write_command(0x200, *left);
	lcd22_write_command(0x201, *top);

	return TRUE;
}

// Reset the draw area to the full LCD screen
static void lcd22_reset_draw_area() {
	// Reset draw window
	lcd22_write_command(0x210, 0x0000);
	lcd22_write_command(0x212, 0x0000);
	lcd22_write_command(0x211, LCD22_WIDTH  - 1);
	lcd22_write_command(0x213, LCD22_HEIGHT - 1);

	// RAM position
	lcd22_write_command(0x200, 0x0000);
	lcd22_write_command(0x201, 0x0000);
}

// -- Public LCD functions

void lcd22_init() {
	lcd22_spio_init();

	// Reset LCD
	LCD22_RST_L();
	_delay_us(5);
	LCD22_RST_H();
	_delay_us(5);

	lcd22_write_command(0x000, 0x0001); // Start Oscillator
	_delay_us(10);

	// Power settings (power mode, voltages)
	lcd22_write_command(0x100, 0x0000);
	lcd22_write_command(0x101, 0x0000);
	lcd22_write_command(0x102, 0x3110);
	lcd22_write_command(0x103, 0xe200);
	lcd22_write_command(0x110, 0x009d);
	lcd22_write_command(0x111, 0x0022);
	lcd22_write_command(0x100, 0x0120);
	_delay_us(20);

	lcd22_write_command(0x100, 0x3120);
	_delay_us(80);

	// Display control (color mode, RAM direction, etc.)
	lcd22_write_command(0x001, 0x0100);
	lcd22_write_command(0x002, 0x0000);
	lcd22_write_command(0x003, 0x1230);
	lcd22_write_command(0x006, 0x0000);
	lcd22_write_command(0x007, 0x0101);
	lcd22_write_command(0x008, 0x0808);
	lcd22_write_command(0x009, 0x0000);
	lcd22_write_command(0x00b, 0x0000);
	lcd22_write_command(0x00c, 0x0000);
	lcd22_write_command(0x00d, 0x0018);

	// LTPS control settings
	lcd22_write_command(0x012, 0x0000);
	lcd22_write_command(0x013, 0x0000);
	lcd22_write_command(0x018, 0x0000);
	lcd22_write_command(0x019, 0x0000);

	// RAM write mask
	lcd22_write_command(0x203, 0x0000);
	lcd22_write_command(0x204, 0x0000);

	// Reset screen and window resolution
	lcd22_write_command(0x210, 0x0000);
	lcd22_write_command(0x211, 0x00ef);
	lcd22_write_command(0x212, 0x0000);
	lcd22_write_command(0x213, 0x013f);
	lcd22_write_command(0x214, 0x0000);
	lcd22_write_command(0x215, 0x0000);
	lcd22_write_command(0x216, 0x0000);
	lcd22_write_command(0x217, 0x0000);

	// Gray scale settings
	lcd22_write_command(0x300, 0x5343);
	lcd22_write_command(0x301, 0x1021);
	lcd22_write_command(0x302, 0x0003);
	lcd22_write_command(0x303, 0x0011);
	lcd22_write_command(0x304, 0x050a);
	lcd22_write_command(0x305, 0x4342);
	lcd22_write_command(0x306, 0x1100);
	lcd22_write_command(0x307, 0x0003);
	lcd22_write_command(0x308, 0x1201);
	lcd22_write_command(0x309, 0x050a);

	// RAM access settings
	lcd22_write_command(0x400, 0x4027);
	lcd22_write_command(0x401, 0x0000);
	lcd22_write_command(0x402, 0x0000);	// First screen drive position (1)
	lcd22_write_command(0x403, 0x013f);	// First screen drive position (2)
	lcd22_write_command(0x404, 0x0000);

	// RAM position
	lcd22_write_command(0x200, 0x0000);
	lcd22_write_command(0x201, 0x0000);

	lcd22_write_command(0x100, 0x7120);
	lcd22_write_command(0x007, 0x0103);
	_delay_us(10);
	lcd22_write_command(0x007, 0x0113);

	_delay_us(20);
}

void lcd22_clear_screen(uint16_t color) {
	lcd22_reset_draw_area();

	lcd22_prepare_write();

	for (uint16_t i = 0; i < LCD22_HEIGHT; i++)
		for (uint16_t j = 0; j < LCD22_WIDTH; j++)
			lcd22_write_data(color);

	lcd22_finish_write();
}

void lcd22_clear_area(int16_t x, int16_t y, int16_t width, int16_t height, uint16_t color) {
	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, width, height, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	for (int16_t i = top; i <= bottom; i++)
		for (int16_t j = left; j <= right; j++)
			lcd22_write_data(color);

	lcd22_finish_write();
}

void lcd22_draw_char(char c, int16_t x, int16_t y,
                     uint16_t foreground_color, uint16_t background_color, lcdfont_t font) {
	uint8_t width, height;
	const uint8_t *charData = lcdfont_getChar(font, c, &width, &height);
	if (!charData)
		return;

	lcd22_draw_bitmap_1bpp_P(charData, x, y, width, height, foreground_color, background_color);
}

void lcd22_draw_string(const char *str, int16_t x, int16_t y,
                       uint16_t foreground_color, uint16_t background_color, lcdfont_t font) {

	/* Draw each character in the string, one after each other.
	 * If the cursor reaches the right edge of the display, start a new line.
	 * If the cursor reaches the bottom edge of the display, continue from the top.
	 */

	while (*str) {
		char c = *str++;

		// Draw a blank for invalid characters
		if ((uint8_t)c >= 128)
			c = 0;

		lcd22_draw_char(c, x, y, foreground_color, background_color, font);

		uint8_t width, height;
		lcdfont_getChar(font, c, &width, &height);

		x += width;
		if (x >= LCD22_WIDTH) {
			x = 0;

			y += height;
			if (y >= LCD22_HEIGHT)
				y = 0;
		}
	}
}

void lcd22_draw_string_P(const char *str, int16_t x, int16_t y,
                         uint16_t foreground_color, uint16_t background_color, lcdfont_t font) {

	/* Draw each character in the string, one after each other.
	 * If the cursor reaches the right edge of the display, start a new line.
	 * If the cursor reaches the bottom edge of the display, continue from the top.
	 */

	while (pgm_read_byte(str)) {
		char c = pgm_read_byte(str++);

		// Draw a blank for invalid characters
		if ((uint8_t)c >= 128)
			c = 0;

		lcd22_draw_char(c, x, y, foreground_color, background_color, font);

		uint8_t width, height;
		lcdfont_getChar(font, c, &width, &height);

		x += width;
		if (x >= LCD22_WIDTH) {
			x = 0;

			y += height;
			if (y >= LCD22_HEIGHT)
				y = 0;
		}
	}
}

void lcd22_draw_dot(int16_t x, int16_t y, int16_t size, uint16_t color) {
	// Center the dot around the given coordinates
	x -= size >> 1;
	y -= size >> 1;

	lcd22_clear_area(x, y, size, size, color);
}

/* Draw a line using Bresenham's Line Algorithm.
 * See https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm for specifics.
 */
void lcd22_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t thickness, uint16_t color) {
	const int16_t dx = ABS(x1 - x0);
	const int16_t dy = ABS(y1 - y0);

	const int16_t sx = (x0 < x1) ? 1 : -1;
	const int16_t sy = (y0 < y1) ? 1 : -1;

	int16_t err = dx - dy;

	while (1) {
		int16_t e2 = 2 * err;

		lcd22_draw_dot(x0, y0, thickness, color);
		if ((x0 == x1) && (y0 == y1))
			break;

		if (e2 > -dy) {
			err -= dy;
			x0  += sx;
		}

		if ((x0 == x1) && (y0 == y1)) {
			lcd22_draw_dot(x0, y0, thickness, color);
			break;
		}

		if (e2 < dx) {
			err += dx;
			y0  += sy;
		}
	}
}

void lcd22_draw_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t thickness, uint16_t color) {
	lcd22_draw_line(x0, y0, x1, y0, thickness, color);
	lcd22_draw_line(x0, y1, x1, y1, thickness, color);
	lcd22_draw_line(x0, y0, x0, y1, thickness, color);
	lcd22_draw_line(x1, y0, x1, y1, thickness, color);
}

void lcd22_draw_filled_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
	if (x0 > x1)
		SWAP(x0, x1);
	if (y0 > y1)
		SWAP(y0, y1);

	lcd22_clear_area(x0, y0, x1 - x0 + 1, y1 - y0 + 1, color);
}

void lcd22_draw_hgradient_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color1, uint16_t color2) {
	if (x0 > x1)
		SWAP(x0, x1);
	if (y0 > y1)
		SWAP(y0, y1);

	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x0, y0, x1 - x0 + 1, y1 - y0 + 1, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	/* Calculate the difference between the two colors and divide by the number of pixels to draw.
	 * This gives us the amount we need to add for each pixel to get a linear gradient.
	 * We shift the color difference by 7 bits for increased precision.
	 */
	const int16_t stepr = (((int16_t)LCD22_COLOR_RED  (color2) - (int16_t)LCD22_COLOR_RED  (color1)) << 7) / (right - left);
	const int16_t stepg = (((int16_t)LCD22_COLOR_GREEN(color2) - (int16_t)LCD22_COLOR_GREEN(color1)) << 7) / (right - left);
	const int16_t stepb = (((int16_t)LCD22_COLOR_BLUE (color2) - (int16_t)LCD22_COLOR_BLUE (color1)) << 7) / (right - left);

	for (int16_t i = top; i <= bottom; i++) {
		uint16_t r = LCD22_COLOR_RED  (color1) << 7;
		uint16_t g = LCD22_COLOR_GREEN(color1) << 7;
		uint16_t b = LCD22_COLOR_BLUE (color1) << 7;

		for (int16_t j = left; j <= right; j++) {
			lcd22_write_data(LCD22_COLOR(r >> 7, g >> 7, b >> 7));
			r += stepr;
			g += stepg;
			b += stepb;
		}
	}

	lcd22_finish_write();
}

void lcd22_draw_vgradient_rectangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color1, uint16_t color2) {
	if (x0 > x1)
		SWAP(x0, x1);
	if (y0 > y1)
		SWAP(y0, y1);

	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x0, y0, x1 - x0 + 1, y1 - y0 + 1, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	/* Calculate the difference between the two colors and divide by the number of pixels to draw.
	 * This gives us the amount we need to add for each pixel to get a linear gradient.
	 * We shift the color difference by 7 bits for increased precision.
	 */
	const int16_t stepr = (((int16_t)LCD22_COLOR_RED  (color2) - (int16_t)LCD22_COLOR_RED  (color1)) << 7) / (bottom - top);
	const int16_t stepg = (((int16_t)LCD22_COLOR_GREEN(color2) - (int16_t)LCD22_COLOR_GREEN(color1)) << 7) / (bottom - top);
	const int16_t stepb = (((int16_t)LCD22_COLOR_BLUE (color2) - (int16_t)LCD22_COLOR_BLUE (color1)) << 7) / (bottom - top);

	uint16_t r = LCD22_COLOR_RED  (color1) << 7;
	uint16_t g = LCD22_COLOR_GREEN(color1) << 7;
	uint16_t b = LCD22_COLOR_BLUE (color1) << 7;
	for (int16_t i = top; i <= bottom; i++) {
		for (int16_t j = left; j <= right; j++)
			lcd22_write_data(LCD22_COLOR(r >> 7, g >> 7, b >> 7));

		r += stepr;
		g += stepg;
		b += stepb;
	}

	lcd22_finish_write();
}

/* Draw a circle using the Midpoint Circle Algorithm.
 * See https://en.wikipedia.org/wiki/Midpoint_circle_algorithm for specifics.
 */
void lcd22_draw_circle(int16_t x0, int16_t y0, int16_t radius, int16_t thickness, uint16_t color) {
	int16_t f = 1 - radius;
	int16_t ddFx = 0;
	int16_t ddFy = -2 * radius;
	int16_t x = 0;
	int16_t y = radius;

	lcd22_draw_dot(x0, y0 + radius, thickness, color);
	lcd22_draw_dot(x0, y0 - radius, thickness, color);
	lcd22_draw_dot(x0 + radius, y0, thickness, color);
	lcd22_draw_dot(x0 - radius, y0, thickness, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddFy += 2;
			f += ddFy;
		}
		x++;
		ddFx += 2;
		f += ddFx + 1;

		lcd22_draw_dot(x0 + x, y0 + y, thickness, color);
		lcd22_draw_dot(x0 - x, y0 + y, thickness, color);
		lcd22_draw_dot(x0 + x, y0 - y, thickness, color);
		lcd22_draw_dot(x0 - x, y0 - y, thickness, color);
		lcd22_draw_dot(x0 + y, y0 + x, thickness, color);
		lcd22_draw_dot(x0 - y, y0 + x, thickness, color);
		lcd22_draw_dot(x0 + y, y0 - x, thickness, color);
		lcd22_draw_dot(x0 - y, y0 - x, thickness, color);
	}
}

void lcd22_draw_filled_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color) {
	for (int16_t y = -radius; y <= radius; y++)
		for (int16_t x = -radius; x <= radius; x++)
			if (((x * x) + (y * y)) <= (radius * radius))
				lcd22_draw_dot(x0 + x, y0 + y, 1, color);
}

void lcd22_draw_hgradient_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color1, uint16_t color2) {
	/* Calculate the difference between the two colors and divide by the number of pixels to draw.
	 * This gives us the amount we need to add for each pixel to get a linear gradient.
	 * We shift the color difference by 7 bits for increased precision.
	 */
	const int16_t stepr = (((int16_t)LCD22_COLOR_RED  (color2) - (int16_t)LCD22_COLOR_RED  (color1)) << 7) / (2 * radius);
	const int16_t stepg = (((int16_t)LCD22_COLOR_GREEN(color2) - (int16_t)LCD22_COLOR_GREEN(color1)) << 7) / (2 * radius);
	const int16_t stepb = (((int16_t)LCD22_COLOR_BLUE (color2) - (int16_t)LCD22_COLOR_BLUE (color1)) << 7) / (2 * radius);

	for (int16_t y = -radius; y <= radius; y++) {
		uint16_t r = LCD22_COLOR_RED  (color1) << 7;
		uint16_t g = LCD22_COLOR_GREEN(color1) << 7;
		uint16_t b = LCD22_COLOR_BLUE (color1) << 7;

		for (int16_t x = -radius; x <= radius; x++) {
			if (((x * x) + (y * y)) <= (radius * radius))
				lcd22_draw_dot(x0 + x, y0 + y, 1, LCD22_COLOR(r >> 7, g >> 7, b >> 7));

			r += stepr;
			g += stepg;
			b += stepb;
		}
	}
}

void lcd22_draw_vgradient_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color1, uint16_t color2) {
	/* Calculate the difference between the two colors and divide by the number of pixels to draw.
	 * This gives us the amount we need to add for each pixel to get a linear gradient.
	 * We shift the color difference by 7 bits for increased precision.
	 */
	const int16_t stepr = (((int16_t)LCD22_COLOR_RED  (color2) - (int16_t)LCD22_COLOR_RED  (color1)) << 7) / (2 * radius);
	const int16_t stepg = (((int16_t)LCD22_COLOR_GREEN(color2) - (int16_t)LCD22_COLOR_GREEN(color1)) << 7) / (2 * radius);
	const int16_t stepb = (((int16_t)LCD22_COLOR_BLUE (color2) - (int16_t)LCD22_COLOR_BLUE (color1)) << 7) / (2 * radius);

	int16_t r = LCD22_COLOR_RED  (color1) << 7;
	int16_t g = LCD22_COLOR_GREEN(color1) << 7;
	int16_t b = LCD22_COLOR_BLUE (color1) << 7;
	for (int16_t y = -radius; y <= radius; y++) {
		for (int16_t x = -radius; x <= radius; x++)
			if (((x * x) + (y * y)) <= (radius * radius))
				lcd22_draw_dot(x0 + x, y0 + y, 1, LCD22_COLOR(r >> 7, g >> 7, b >> 7));

		r += stepr;
		g += stepg;
		b += stepb;
	}
}

static void lcd22_draw_bitmap_paletted(const uint8_t *bitmap, const uint16_t *palette,
                                       int16_t x, int16_t y, int16_t width, int16_t height, uint8_t bits) {

	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, width, height, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	// Go through each row of the bitmap data, always starting with a new byte
	for (uint16_t j = top; j <= bottom; j++) {
		uint8_t bitPos = 255, p = 0x00;

		for (uint16_t i = left; i <= right; i++) {
			// Fetch a new byte if necessary
			if (bitPos++ >= ((8 / bits) - 1)) {
				bitPos = 0;
				p = *bitmap++;
			}

			// Take n bits as an index into the palette and draw the pixel
			lcd22_write_data(palette[p >> (8 - bits)]);
			p <<= bits;
		}
	}

	lcd22_finish_write();
}

static void lcd22_draw_bitmap_paletted_P(const uint8_t *bitmap, const uint16_t *palette,
                                         int16_t x, int16_t y, int16_t width, int16_t height, uint8_t bits) {

	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, width, height, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	// Go through each row of the bitmap data, always starting with a new byte
	for (uint16_t j = top; j <= bottom; j++) {
		uint8_t bitPos = 255, p = 0x00;

		for (uint16_t i = left; i <= right; i++) {
			// Fetch a new byte if necessary
			if (bitPos++ >= ((8 / bits) - 1)) {
				bitPos = 0;
				p = pgm_read_byte(bitmap++);
			}

			// Take n bits as an index into the palette and draw the pixel
			lcd22_write_data(palette[p >> (8 - bits)]);
			p <<= bits;
		}
	}

	lcd22_finish_write();
}

static void lcd22_draw_bitmap_paletted_PP(const uint8_t *bitmap, const uint16_t *palette,
                                          int16_t x, int16_t y, int16_t width, int16_t height, uint8_t bits) {

	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, width, height, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	// Go through each row of the bitmap data, always starting with a new byte
	for (uint16_t j = top; j <= bottom; j++) {
		uint8_t bitPos = 255, p = 0x00;

		for (uint16_t i = left; i <= right; i++) {
			// Fetch a new byte if necessary
			if (bitPos++ >= ((8 / bits) - 1)) {
				bitPos = 0;
				p = pgm_read_byte(bitmap++);
			}

			// Take n bits as an index into the palette and draw the pixel
			lcd22_write_data(pgm_read_word(&palette[p >> (8 - bits)]));
			p <<= bits;
		}
	}

	lcd22_finish_write();
}

void lcd22_draw_bitmap_1bpp(const uint8_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height,
                            uint16_t foreground_color, uint16_t background_color) {

	const uint16_t palette[2] = {background_color, foreground_color};

	lcd22_draw_bitmap_paletted(bitmap, palette, x, y, width, height, 1);
}

void lcd22_draw_bitmap_1bpp_P(const uint8_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height,
                              uint16_t foreground_color, uint16_t background_color) {

	const uint16_t palette[2] = {background_color, foreground_color};

	lcd22_draw_bitmap_paletted_P(bitmap, palette, x, y, width, height, 1);
}

void lcd22_draw_bitmap_1bpp_PP(const uint8_t *bitmap, const uint16_t *palette,
                               int16_t x, int16_t y, int16_t width, int16_t height) {

	lcd22_draw_bitmap_paletted_PP(bitmap, palette, x, y, width, height, 1);
}

void lcd22_draw_bitmap_16bpp(const uint16_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height) {
	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, width, height, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	for (int16_t i = top; i <= bottom; i++)
		for (int16_t j = left; j <= right; j++)
			lcd22_write_data(*bitmap++);

	lcd22_finish_write();
}

void lcd22_draw_bitmap_16bpp_P(const uint16_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height) {
	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, width, height, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	for (int16_t i = top; i <= bottom; i++)
		for (int16_t j = left; j <= right; j++)
			lcd22_write_data(pgm_read_word(bitmap++));

	lcd22_finish_write();
}

void lcd22_draw_bitmap_24bpp(const uint8_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height) {
	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, width, height, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	for (int16_t i = top; i <= bottom; i++) {
		for (int16_t j = left; j <= right; j++) {
			const uint8_t r = *bitmap++;
			const uint8_t g = *bitmap++;
			const uint8_t b = *bitmap++;

			lcd22_write_data(LCD22_COLOR(r, g, b));
		}
	}

	lcd22_finish_write();
}

void lcd22_draw_bitmap_24bpp_P(const uint8_t *bitmap, int16_t x, int16_t y, int16_t width, int16_t height) {
	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, width, height, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	for (int16_t i = top; i <= bottom; i++) {
		for (int16_t j = left; j <= right; j++) {
			const uint8_t r = pgm_read_byte(bitmap++);
			const uint8_t g = pgm_read_byte(bitmap++);
			const uint8_t b = pgm_read_byte(bitmap++);

			lcd22_write_data(LCD22_COLOR(r, g, b));
		}
	}

	lcd22_finish_write();
}

void lcd22_draw_bitmap_2bpp(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height) {
	lcd22_draw_bitmap_paletted(bitmap, palette, x, y, width, height, 2);
}

void lcd22_draw_bitmap_2bpp_P(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height) {
	lcd22_draw_bitmap_paletted_P(bitmap, palette, x, y, width, height, 2);
}

void lcd22_draw_bitmap_2bpp_PP(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height) {
	lcd22_draw_bitmap_paletted_PP(bitmap, palette, x, y, width, height, 2);
}

void lcd22_draw_bitmap_4bpp(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height) {
	lcd22_draw_bitmap_paletted(bitmap, palette, x, y, width, height, 4);
}

void lcd22_draw_bitmap_4bpp_P(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height) {
	lcd22_draw_bitmap_paletted_P(bitmap, palette, x, y, width, height, 4);
}

void lcd22_draw_bitmap_4bpp_PP(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height) {
	lcd22_draw_bitmap_paletted_PP(bitmap, palette, x, y, width, height, 4);
}

void lcd22_draw_bitmap_8bpp(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height) {
	lcd22_draw_bitmap_paletted(bitmap, palette, x, y, width, height, 8);
}

void lcd22_draw_bitmap_8bpp_P(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height) {
	lcd22_draw_bitmap_paletted_P(bitmap, palette, x, y, width, height, 8);
}

void lcd22_draw_bitmap_8bpp_PP(const uint8_t *bitmap, const uint16_t *palette, int16_t x, int16_t y, int16_t width, int16_t height) {
	lcd22_draw_bitmap_paletted_PP(bitmap, palette, x, y, width, height, 8);
}

// -- Touch screen utility macros

#define LCD22_TOUCH_COMMAND_READ_X  0xD0
#define LCD22_TOUCH_COMMAND_READ_Y  0x90

// -- Internal touch screen support functions --

// Read ADC value from the touch screen controller
static int16_t lcd22_touch_read_adc(uint8_t command) {
	LCD22_CS_H();
	LCD22_TOUCH_CS_L();

	_delay_us(2);
	lcd22_spio_send(command);
	_delay_us(2);

	uint16_t numh = lcd22_spio_send(0x00);
	uint16_t numl = lcd22_spio_send(0x00);

	LCD22_TOUCH_CS_H();

	return ((numh << 8) + numl) >> 4;
}

// Read ADC value from the touch screen controller several times and return an average (ignoring outliers)
#define LCD22_TOUCH_AVERAGE_READ    10
#define LCD22_TOUCH_AVERAGE_IGNORE   4
#define LCD22_TOUCH_AVERAGE_USE    (LCD22_TOUCH_AVERAGE_READ - (2*LCD22_TOUCH_AVERAGE_IGNORE))
static int16_t lcd22_touch_read_adc_average(uint8_t command) {
	int16_t buf[LCD22_TOUCH_AVERAGE_READ];

	// Read the ADC LCD22_TOUCH_AVERAGE_READ times
	for (uint8_t i = 0; i < LCD22_TOUCH_AVERAGE_READ; i++)
		buf[i] = lcd22_touch_read_adc(command);

	// Sort the read ADC values
	for (uint8_t i = 0; i < LCD22_TOUCH_AVERAGE_READ-1; i++)
		for (uint8_t j = i + 1; j < LCD22_TOUCH_AVERAGE_READ; j++)
			if (buf[i] > buf[j])
				SWAP(buf[i], buf[j]);

	// Average over the middle values
	int16_t avg = 0;
	for (uint8_t i = 0; i < LCD22_TOUCH_AVERAGE_USE; i++)
		avg += buf[LCD22_TOUCH_AVERAGE_IGNORE + i];
	avg /= LCD22_TOUCH_AVERAGE_USE;

	return avg;
}

static bool lcd22_touch_read_adc_position(int16_t *x, int16_t *y) {
	*x = lcd22_touch_read_adc_average(LCD22_TOUCH_COMMAND_READ_X);
	*y = lcd22_touch_read_adc_average(LCD22_TOUCH_COMMAND_READ_Y);

	if ((*x < 100) || (*y < 100))
		return FALSE;

	return TRUE;
}

// Read the averaged position twice and check if the values are within a certain error range
#define LCD22_TOUCH_ERROR_RANGE 50
static bool lcd22_touch_read_adc_position_safe(int16_t *x, int16_t *y) {
	int16_t x1, y1, x2, y2;

	if (!lcd22_touch_read_adc_position(&x1, &y1))
		return FALSE;
	if (!lcd22_touch_read_adc_position(&x2, &y2))
		return FALSE;

	if ((ABS(x1 - x2) >= LCD22_TOUCH_ERROR_RANGE) || (ABS(y1 - y2) >= LCD22_TOUCH_ERROR_RANGE))
		return FALSE;

	*x = (x1 + x2) / 2;
	*y = (y1 + y2) / 2;
	return TRUE;
}

static void lcd22_touch_convert_adc_position_to_lcd_position(int16_t adc_x, int16_t adc_y, int16_t *lcd_x, int16_t *lcd_y) {
	*lcd_x = (((int32_t)adc_x - 100) * LCD22_WIDTH ) / 1800;
	*lcd_y = (((int32_t)adc_y - 100) * LCD22_HEIGHT) / 1300;
}

// -- Public touch screen functions --

bool lcd22_get_touch(int16_t *x, int16_t *y) {
	int16_t adc_x, adc_y;
	if (!lcd22_touch_read_adc_position_safe(&adc_x, &adc_y))
		return FALSE;

	lcd22_touch_convert_adc_position_to_lcd_position(adc_x, adc_y, x, y);
	return TRUE;
}
