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
#include <util/delay.h>

#include "lcd22.h"

#include "util.h"

// Image data for the lower 128 (7-bit) ASCII characters
static const uint8_t lcd22_ascii[] = {
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // 0x00
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,
	0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x08,0x08,0x08,0x08,0x08,0x08,0x08,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x18,0x3C,0x7E,0x7E,0x7E,0x3C,0x18,0x00,0x00,0x00,0x00,0x00,0x00,
	0xFF,0xFF,0xFF,0xE7,0xC3,0x81,0x81,0x81,0xC3,0xE7,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x1F,0x05,0x05,0x09,0x09,0x10,0x10,0x38,0x44,0x44,0x44,0x38,0x00,0x00,0x00,
	0x00,0x1C,0x22,0x22,0x22,0x1C,0x08,0x08,0x7F,0x08,0x08,0x08,0x08,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x03,0x1D,0x11,0x13,0x1D,0x11,0x11,0x11,0x13,0x17,0x36,0x70,0x60,0x00,0x00,0x00,
	0x08,0x08,0x5D,0x22,0x22,0x22,0x63,0x22,0x22,0x22,0x5D,0x08,0x08,0x00,0x00,0x00,
	0x08,0x08,0x08,0x08,0x08,0x08,0x08,0xFF,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00, // 0x10
	0x00,0x01,0x03,0x07,0x0F,0x1F,0x3F,0x7F,0x3F,0x1F,0x0F,0x07,0x03,0x01,0x00,0x00,
	0x08,0x1C,0x2A,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x2A,0x1C,0x08,0x00,0x00,
	0x00,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x24,0x00,0x00,0x24,0x24,0x00,0x00,
	0x00,0x1F,0x25,0x45,0x45,0x45,0x25,0x1D,0x05,0x05,0x05,0x05,0x05,0x00,0x00,0x00,
	0x08,0x08,0x08,0x08,0x08,0x08,0x08,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,
	0x08,0x08,0x08,0x08,0x08,0x08,0x08,0xF8,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,
	0x08,0x1C,0x2A,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x00,
	0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x0F,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,
	0x00,0x00,0x00,0x00,0x00,0x04,0x02,0x7F,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x10,0x20,0x7F,0x20,0x10,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // 0x20
	0x00,0x00,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x18,0x18,0x00,0x00,0x00,
	0x12,0x36,0x24,0x48,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x24,0x24,0x24,0xFE,0x48,0x48,0x48,0xFE,0x48,0x48,0x48,0x00,0x00,0x00,
	0x00,0x10,0x38,0x54,0x54,0x50,0x30,0x18,0x14,0x14,0x54,0x54,0x38,0x10,0x10,0x00,
	0x00,0x00,0x44,0xA4,0xA8,0xA8,0xA8,0x54,0x1A,0x2A,0x2A,0x2A,0x44,0x00,0x00,0x00,
	0x00,0x00,0x30,0x48,0x48,0x48,0x50,0x6E,0xA4,0x94,0x88,0x89,0x76,0x00,0x00,0x00,
	0x60,0x60,0x20,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x02,0x04,0x08,0x08,0x10,0x10,0x10,0x10,0x10,0x10,0x08,0x08,0x04,0x02,0x00,0x00,
	0x40,0x20,0x10,0x10,0x08,0x08,0x08,0x08,0x08,0x08,0x10,0x10,0x20,0x40,0x00,0x00,
	0x00,0x00,0x00,0x10,0x10,0xD6,0x38,0x38,0xD6,0x10,0x10,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x10,0x10,0x10,0x10,0xFE,0x10,0x10,0x10,0x10,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x60,0x20,0xC0,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x00,
	0x00,0x01,0x02,0x02,0x04,0x04,0x08,0x08,0x10,0x10,0x20,0x20,0x40,0x40,0x00,0x00,
	0x00,0x00,0x18,0x24,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x18,0x00,0x00,0x00, // 0x30
	0x00,0x00,0x10,0x70,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,0x00,
	0x00,0x00,0x3C,0x42,0x42,0x42,0x04,0x04,0x08,0x10,0x20,0x42,0x7E,0x00,0x00,0x00,
	0x00,0x00,0x3C,0x42,0x42,0x04,0x18,0x04,0x02,0x02,0x42,0x44,0x38,0x00,0x00,0x00,
	0x00,0x00,0x04,0x0C,0x14,0x24,0x24,0x44,0x44,0x7E,0x04,0x04,0x1E,0x00,0x00,0x00,
	0x00,0x00,0x7E,0x40,0x40,0x40,0x58,0x64,0x02,0x02,0x42,0x44,0x38,0x00,0x00,0x00,
	0x00,0x00,0x1C,0x24,0x40,0x40,0x58,0x64,0x42,0x42,0x42,0x24,0x18,0x00,0x00,0x00,
	0x00,0x00,0x7E,0x44,0x44,0x08,0x08,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00,
	0x00,0x00,0x3C,0x42,0x42,0x42,0x24,0x18,0x24,0x42,0x42,0x42,0x3C,0x00,0x00,0x00,
	0x00,0x00,0x18,0x24,0x42,0x42,0x42,0x26,0x1A,0x02,0x02,0x24,0x38,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x00,0x10,0x10,0x20,0x00,
	0x00,0x00,0x02,0x04,0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x04,0x02,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x40,0x20,0x10,0x08,0x04,0x02,0x04,0x08,0x10,0x20,0x40,0x00,0x00,0x00,
	0x00,0x00,0x3C,0x42,0x42,0x62,0x02,0x04,0x08,0x08,0x00,0x18,0x18,0x00,0x00,0x00,
	0x00,0x00,0x38,0x44,0x5A,0xAA,0xAA,0xAA,0xAA,0xB4,0x42,0x44,0x38,0x00,0x00,0x00,  // 0x40
	0x00,0x00,0x10,0x10,0x18,0x28,0x28,0x24,0x3C,0x44,0x42,0x42,0xE7,0x00,0x00,0x00,
	0x00,0x00,0xF8,0x44,0x44,0x44,0x78,0x44,0x42,0x42,0x42,0x44,0xF8,0x00,0x00,0x00,
	0x00,0x00,0x3E,0x42,0x42,0x80,0x80,0x80,0x80,0x80,0x42,0x44,0x38,0x00,0x00,0x00,
	0x00,0x00,0xF8,0x44,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x44,0xF8,0x00,0x00,0x00,
	0x00,0x00,0xFC,0x42,0x48,0x48,0x78,0x48,0x48,0x40,0x42,0x42,0xFC,0x00,0x00,0x00,
	0x00,0x00,0xFC,0x42,0x48,0x48,0x78,0x48,0x48,0x40,0x40,0x40,0xE0,0x00,0x00,0x00,
	0x00,0x00,0x3C,0x44,0x44,0x80,0x80,0x80,0x8E,0x84,0x44,0x44,0x38,0x00,0x00,0x00,
	0x00,0x00,0xE7,0x42,0x42,0x42,0x42,0x7E,0x42,0x42,0x42,0x42,0xE7,0x00,0x00,0x00,
	0x00,0x00,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,0x00,
	0x00,0x00,0x3E,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x88,0xF0,0x00,
	0x00,0x00,0xEE,0x44,0x48,0x50,0x70,0x50,0x48,0x48,0x44,0x44,0xEE,0x00,0x00,0x00,
	0x00,0x00,0xE0,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x42,0xFE,0x00,0x00,0x00,
	0x00,0x00,0xEE,0x6C,0x6C,0x6C,0x6C,0x54,0x54,0x54,0x54,0x54,0xD6,0x00,0x00,0x00,
	0x00,0x00,0xC7,0x62,0x62,0x52,0x52,0x4A,0x4A,0x4A,0x46,0x46,0xE2,0x00,0x00,0x00,
	0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x44,0x38,0x00,0x00,0x00,
	0x00,0x00,0xFC,0x42,0x42,0x42,0x42,0x7C,0x40,0x40,0x40,0x40,0xE0,0x00,0x00,0x00, // 0x50
	0x00,0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0xB2,0xCA,0x4C,0x38,0x06,0x00,0x00,
	0x00,0x00,0xFC,0x42,0x42,0x42,0x7C,0x48,0x48,0x44,0x44,0x42,0xE3,0x00,0x00,0x00,
	0x00,0x00,0x3E,0x42,0x42,0x40,0x20,0x18,0x04,0x02,0x42,0x42,0x7C,0x00,0x00,0x00,
	0x00,0x00,0xFE,0x92,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00,0x00,
	0x00,0x00,0xE7,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,0x00,
	0x00,0x00,0xE7,0x42,0x42,0x44,0x24,0x24,0x28,0x28,0x18,0x10,0x10,0x00,0x00,0x00,
	0x00,0x00,0xD6,0x92,0x92,0x92,0x92,0xAA,0xAA,0x6C,0x44,0x44,0x44,0x00,0x00,0x00,
	0x00,0x00,0xE7,0x42,0x24,0x24,0x18,0x18,0x18,0x24,0x24,0x42,0xE7,0x00,0x00,0x00,
	0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00,0x00,
	0x00,0x00,0x7E,0x84,0x04,0x08,0x08,0x10,0x20,0x20,0x42,0x42,0xFC,0x00,0x00,0x00,
	0x1E,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x1E,0x00,0x00,
	0x00,0x40,0x40,0x20,0x20,0x10,0x10,0x10,0x08,0x08,0x04,0x04,0x04,0x02,0x02,0x00,
	0x78,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x78,0x00,0x00,
	0x1C,0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,
	0x60,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // 0x60
	0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x42,0x1E,0x22,0x42,0x42,0x3F,0x00,0x00,0x00,
	0x00,0x00,0xC0,0x40,0x40,0x40,0x58,0x64,0x42,0x42,0x42,0x64,0x58,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x22,0x40,0x40,0x40,0x22,0x1C,0x00,0x00,0x00,
	0x00,0x00,0x06,0x02,0x02,0x02,0x1E,0x22,0x42,0x42,0x42,0x26,0x1B,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x42,0x7E,0x40,0x40,0x42,0x3C,0x00,0x00,0x00,
	0x00,0x00,0x0F,0x11,0x10,0x10,0x7E,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x44,0x44,0x38,0x40,0x3C,0x42,0x42,0x3C,0x00,
	0x00,0x00,0xC0,0x40,0x40,0x40,0x5C,0x62,0x42,0x42,0x42,0x42,0xE7,0x00,0x00,0x00,
	0x00,0x00,0x30,0x30,0x00,0x00,0x70,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,0x00,
	0x00,0x00,0x0C,0x0C,0x00,0x00,0x1C,0x04,0x04,0x04,0x04,0x04,0x04,0x44,0x78,0x00,
	0x00,0x00,0xC0,0x40,0x40,0x40,0x4E,0x48,0x50,0x68,0x48,0x44,0xEE,0x00,0x00,0x00,
	0x00,0x00,0x70,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x49,0x49,0x49,0x49,0x49,0xED,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xDC,0x62,0x42,0x42,0x42,0x42,0xE7,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x42,0x42,0x42,0x42,0x42,0x3C,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xD8,0x64,0x42,0x42,0x42,0x44,0x78,0x40,0xE0,0x00, // 0x70
	0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0x22,0x42,0x42,0x42,0x22,0x1E,0x02,0x07,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xEE,0x32,0x20,0x20,0x20,0x20,0xF8,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x42,0x40,0x3C,0x02,0x42,0x7C,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x10,0x10,0x7C,0x10,0x10,0x10,0x10,0x10,0x0C,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xC6,0x42,0x42,0x42,0x42,0x46,0x3B,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xE7,0x42,0x24,0x24,0x28,0x10,0x10,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xD7,0x92,0x92,0xAA,0xAA,0x44,0x44,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x6E,0x24,0x18,0x18,0x18,0x24,0x76,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0xE7,0x42,0x24,0x24,0x28,0x18,0x10,0x10,0xE0,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x7E,0x44,0x08,0x10,0x10,0x22,0x7E,0x00,0x00,0x00,
	0x03,0x04,0x04,0x04,0x04,0x04,0x08,0x04,0x04,0x04,0x04,0x04,0x04,0x03,0x00,0x00,
	0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,
	0x60,0x10,0x10,0x10,0x10,0x10,0x08,0x10,0x10,0x10,0x10,0x10,0x10,0x60,0x00,0x00,
	0x4C,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

// -- Utility macros --

#define LCD22_RST_H()        PORTB |=  (1<<PB6)
#define LCD22_RST_L()        PORTB &= ~(1<<PB6)

#define LCD22_RS_H()         PORTB |=  (1<<PB5)
#define LCD22_RS_L()         PORTB &= ~(1<<PB5)

#define LCD22_CS_H()         PORTD |=  (1<<PD6)
#define LCD22_CS_L()         PORTD &= ~(1<<PD6)

#define LCD22_TOUCH_CS_H()   PORTB |=  (1<<PB4)
#define LCD22_TOUCH_CS_L()   PORTB &= ~(1<<PB4)

// -- Internal support functions --

// Initialize the SPI I/O
static void lcd22_spio_init(void) {
	// Set direction of LCD pins
	DDRB = (DDRB & ~0x7F) | 0x77;
	DDRD |= 0x40;

	// Clear LCD pins / pull-up register setting
	PORTB &= ~0x7F;
	PORTD &= ~0x40;

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

	// Clamp to valid area
	*left   = CLIP(x             , 0, LCD22_WIDTH  - 1);
	*top    = CLIP(y             , 0, LCD22_HEIGHT - 1);
	*right  = CLIP(x + width  - 1, 0, LCD22_WIDTH  - 1);
	*bottom = CLIP(y + height - 1, 0, LCD22_HEIGHT - 1);

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
	uint16_t i, j;

	lcd22_reset_draw_area();

	lcd22_prepare_write();

	for (i = 0; i < LCD22_HEIGHT; i++)
		for (j = 0; j < LCD22_WIDTH; j++)
			lcd22_write_data(color);

	lcd22_finish_write();
}

void lcd22_clear_area(int16_t x, int16_t y, int16_t width, int16_t height, uint16_t color) {
	int16_t i, j;

	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, width, height, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	for (i = top; i <= bottom; i++)
		for (j = left; j <= right; j++)
			lcd22_write_data(color);

	lcd22_finish_write();
}

void lcd22_draw_char(char c, int16_t x, int16_t y, uint16_t foreground_color, uint16_t background_color) {
	uint8_t i, j, b;
	const uint8_t *p;

	int16_t left, top, right, bottom;
	if (!lcd22_set_draw_area(x, y, LCD22_CHAR_WIDTH, LCD22_CHAR_HEIGHT, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	// Make sure we don't print invalid / unsupported characters
	if ((uint8_t)c >= 128)
		c = 0;

	/* Go through the image data for the selected character:
	 * Each character consists of 16 rows of 8 pixels. Each row is one byte; each pixel is one bit.
	 * If the bit is set, it belongs to the character, so draw the foreground color.
	 * If the bit is not set, it doesn't belong to the character, so draw the background color.
	 */

	p = lcd22_ascii + c * 16;

	for (j = 0; j < 16; j++) {
		b = *(p + j);

		for (i = 0; i < 8; i++) {
			lcd22_write_data((b & 0x80) ? foreground_color : background_color);

			b <<= 1;
		}
	}

	lcd22_finish_write();
}

void lcd22_draw_string(const char *str, int16_t x, int16_t y, uint16_t foreground_color, uint16_t background_color) {
	/* Draw each character in the string, one after each other.
	 * If the cursor reaches the right edge of the display, start a new line.
	 * If the cursor reaches the bottom edge of the display, continue from the top.
	 */

	while (*str) {
		lcd22_draw_char(*str++, x, y, foreground_color, background_color);

		x += LCD22_CHAR_WIDTH;
		if (x >= LCD22_WIDTH) {
			x = 0;

			y += LCD22_CHAR_HEIGHT;
			if (y >= LCD22_HEIGHT)
				y = 0;
		}
	}
}

void lcd22_draw_dot(int16_t x, int16_t y, uint16_t color) {
	int16_t left, top, right, bottom, count;
	if (!lcd22_set_draw_area(x, y, 1, 1, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	count = (right - left + 1) * (bottom - top + 1);
	while (count-- > 0)
		lcd22_write_data(color);

	lcd22_finish_write();
}

void lcd22_draw_big_dot(int16_t x, int16_t y, uint16_t color) {
	int16_t left, top, right, bottom, count;
	if (!lcd22_set_draw_area(x -1 , y - 1, 3, 3, &left, &top, &right, &bottom))
		return;

	lcd22_prepare_write();

	count = (right - left + 1) * (bottom - top + 1);
	while (count-- > 0)
		lcd22_write_data(color);

	lcd22_finish_write();
}

/* Draw a line using Bresenham's Line Algorithm.
 * See https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm for specifics.
 */
static void lcd22_draw_line_internal(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color,
                                     void (*plot)(int16_t, int16_t,uint16_t)) {

	const int16_t dx = ABS(x1 - x0);
	const int16_t dy = ABS(y1 - y0);

	const int16_t sx = (x0 < x1) ? 1 : -1;
	const int16_t sy = (y0 < y1) ? 1 : -1;

	int16_t err = dx - dy;

	while (1) {
		int16_t e2 = 2 * err;

		(*plot)(x0, y0, color);
		if ((x0 == x1) && (y0 == y1))
			break;

		if (e2 > -dy) {
			err -= dy;
			x0  += sx;
		}

		if ((x0 == x1) && (y0 == y1)) {
			(*plot)(x0, y0, color);
			break;
		}

		if (e2 < dx) {
			err += dx;
			y0  += sy;
		}
	}
}

void lcd22_draw_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
	lcd22_draw_line_internal(x0, y0, x1, y1, color, &lcd22_draw_dot);
}

void lcd22_draw_thick_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
	lcd22_draw_line_internal(x0, y0, x1, y1, color, &lcd22_draw_big_dot);
}

/* Draw a circle using the Midpoint Circle Algorithm.
 * See https://en.wikipedia.org/wiki/Midpoint_circle_algorithm for specifics.
 */
static void lcd22_draw_circle_internal(int16_t x0, int16_t y0, int16_t radius, uint16_t color,
                                       void (*plot)(int16_t, int16_t,uint16_t)) {

	int16_t f = 1 - radius;
	int16_t ddFx = 0;
	int16_t ddFy = -2 * radius;
	int16_t x = 0;
	int16_t y = radius;

	(*plot)(x0, y0 + radius, color);
	(*plot)(x0, y0 - radius, color);
	(*plot)(x0 + radius, y0, color);
	(*plot)(x0 - radius, y0, color);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddFy += 2;
			f += ddFy;
		}
		x++;
		ddFx += 2;
		f += ddFx + 1;

		(*plot)(x0 + x, y0 + y, color);
		(*plot)(x0 - x, y0 + y, color);
		(*plot)(x0 + x, y0 - y, color);
		(*plot)(x0 - x, y0 - y, color);
		(*plot)(x0 + y, y0 + x, color);
		(*plot)(x0 - y, y0 + x, color);
		(*plot)(x0 + y, y0 - x, color);
		(*plot)(x0 - y, y0 - x, color);
	}
}

void lcd22_draw_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color) {
	lcd22_draw_circle_internal(x0, y0, radius, color, &lcd22_draw_dot);
}

void lcd22_draw_thick_circle(int16_t x0, int16_t y0, int16_t radius, uint16_t color) {
	lcd22_draw_circle_internal(x0, y0, radius, color, &lcd22_draw_big_dot);
}