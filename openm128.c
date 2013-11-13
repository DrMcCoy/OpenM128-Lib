/* openm128 - Test code for the OpenM128-Lib
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

/** @file openm128.c
 *  Warning: This is very hacky and very ugly test code. It is neither meant
 *           as production code, nor to show the quality of the OpenM128-Lib.
 */

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "openm128/util.h"
#include "openm128/keys.h"
#include "openm128/leds.h"
#include "openm128/joystick.h"
#include "openm128/lcd22.h"
#include "openm128/lcd12864st.h"
#include "openm128/keypadad.h"
#include "openm128/keypad4x4.h"
#include "openm128/keypad5io.h"
#include "openm128/keypadtouch.h"
#include "openm128/ds18b20.h"
#include "openm128/segment.h"
#include "openm128/pcf8563.h"
#include "openm128/usart0.h"
#include "openm128/at24cxx.h"
#include "openm128/mag3110.h"
#include "openm128/pcf8574.h"
#include "openm128/pcf8591.h"
#include "openm128/adc.h"
#include "openm128/i2c.h"
#include "openm128/k9fxx08x0c.h"
#include "openm128/vs1003b.h"

static const uint8_t mask_keys = KEYS_MASK(FALSE, TRUE, TRUE, TRUE);
static const uint8_t mask_leds = LEDS_MASK(TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, TRUE);

static const char txt[] PROGMEM = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur lacinia in " \
                                  "odio eu lobortis. Aenean sed varius mi. Maecenas mattis nulla sit amet orci "   \
                                  "bibendum tempor. Phasellus id nisl non nulla dictum feugiat et eget libero. "   \
                                  "In sodales velit eget ligula mollis, sed sollicitudin diam adipiscing. "       \
                                  "Quisque tincidunt porta purus. Donec in felis eget ipsum ullamcorper ultricies...";

#define TESTMETHOD 16
void testLCD22();      //  0
void testTouch();      //  1
void testTouchD();     //  2
void testJoystick();   //  3
void testKeypadAD();   //  4
void testKeypad4x4();  //  5
void testKeypad5io();  //  6
void testDS18B20();    //  7
void testSegment();    //  8
void testLCD22Font();  //  9
void testLCD12864ST(); // 10
void testAT24();       // 11
void testMAG3110();    // 12
void testPCF8574();    // 13
void testPCF8591();    // 14
void testNAND();       // 15
void testMP3();        // 16

#define SQR(x) ((x)*(x))
#define DIFF(a, b) (((int32_t)(a)) - ((int32_t)(b)))

#if TESTMETHOD == 5
	keypad4x4_t keypad4x4;
#endif

#if TESTMETHOD == 6
	keypad5io_t keypad5io;
#endif

#if TESTMETHOD == 8
	segment_t segment;
#endif

#if TESTMETHOD == 10
	bool valid_start_time = FALSE;
	keypadtouch_t keypadtouch;
#endif

int main(void) {
#if TESTMETHOD != 8 && TESTMETHOD < 10
	keys_init(mask_keys);
	leds_init(mask_leds);
	leds_set_mask(mask_leds, 0x00);
#endif

#if TESTMETHOD == 3
	joystick_init();
#endif

#if TESTMETHOD == 5
	keypad4x4_init(&keypad4x4, generic_io_create(kPortC, 0),
	                           generic_io_create(kPortC, 1),
	                           generic_io_create(kPortC, 2),
	                           generic_io_create(kPortC, 3),
	                           generic_io_create(kPortC, 4),
	                           generic_io_create(kPortC, 5),
	                           generic_io_create(kPortC, 6),
	                           generic_io_create(kPortC, 7));
#endif

#if TESTMETHOD == 6
	keypad5io_init(&keypad5io, generic_io_create(kPortC, 0),
	                           generic_io_create(kPortC, 1),
	                           generic_io_create(kPortC, 2),
	                           generic_io_create(kPortC, 3),
	                           generic_io_create(kPortC, 4));
#endif

#if TESTMETHOD == 8
	segment_init(&segment, generic_io_create(kPortA, 0),
                           generic_io_create(kPortA, 1),
                           generic_io_create(kPortA, 2),
                           generic_io_create(kPortA, 3),
                           generic_io_create(kPortA, 4),
                           generic_io_create(kPortC, 0),
                           generic_io_create(kPortC, 1),
                           generic_io_create(kPortC, 2),
                           generic_io_create(kPortC, 3),
                           generic_io_create(kPortC, 4),
                           generic_io_create(kPortC, 5),
                           generic_io_create(kPortC, 6),
                           generic_io_create(kPortC, 7));
#endif

#if TESTMETHOD == 10
	keypadtouch_init(&keypadtouch);
#endif

#if TESTMETHOD >= 0 && TESTMETHOD <= 9
	_delay_ms(200);
	lcd22_init();
	_delay_ms(200);

	lcd22_clear_screen(LCD22_WHITE);
	lcd22_clear_screen(LCD22_GREEN);
#elif TESTMETHOD == 10

	pcf8563_init_wait();
	valid_start_time = pcf8563_has_valid_time();
	if (!valid_start_time)
		pcf8563_set(pcf8563_time_compile());

	keypadtouch_recalibrate(&keypadtouch);
	lcd12864st_init();
	lcd12864st_print_centered(1, "LCD");
	lcd12864st_refresh();
	_delay_ms(5000);
	lcd12864st_clear();
	lcd12864st_refresh();
	_delay_ms(1000);
#elif TESTMETHOD == 11
	i2c_init();
	lcd12864st_init();
	lcd12864st_print_centered(1, "LCD");
	lcd12864st_refresh();
	_delay_ms(1000);
	lcd12864st_clear();
	lcd12864st_refresh();
#elif TESTMETHOD == 12

#endif // TESTMETHOD >= 0 && TESTMETHOD <= 9

	while (1) {
#if TESTMETHOD >= 0 && TESTMETHOD <= 9
		if (keys_get(1))
			lcd22_init();
		if (keys_get(2))
			lcd22_clear_screen(LCD22_GREEN);
#endif // TESTMETHOD >= 0 && TESTMETHOD <= 9

#if   TESTMETHOD == 0
		testLCD22();
#elif TESTMETHOD == 1
		testTouch();
#elif TESTMETHOD == 2
		testTouchD();
#elif TESTMETHOD == 3
		testJoystick();
#elif TESTMETHOD == 4
		testKeypadAD();
#elif TESTMETHOD == 5
		testKeypad4x4();
#elif TESTMETHOD == 6
		testKeypad5io();
#elif TESTMETHOD == 7
		testDS18B20();
#elif TESTMETHOD == 8
		testSegment();
#elif TESTMETHOD == 9
		testLCD22Font();
#elif TESTMETHOD == 10
		testLCD12864ST();
#elif TESTMETHOD == 11
		testAT24();
#elif TESTMETHOD == 12
		testMAG3110();
#elif TESTMETHOD == 13
		testPCF8574();
#elif TESTMETHOD == 14
		testPCF8591();
#elif TESTMETHOD == 15
		testNAND();
#elif TESTMETHOD == 16
		testMP3();
#endif
	}
}

#if TESTMETHOD == 0

void testLCD22() {
	int x0 = rand() % LCD22_WIDTH;
	int x1 = rand() % LCD22_WIDTH;
	int x2 = rand() % LCD22_WIDTH;
	int x3 = rand() % LCD22_WIDTH;
	int y0 = rand() % LCD22_HEIGHT;
	int y1 = rand() % LCD22_HEIGHT;
	int y2 = rand() % LCD22_HEIGHT;
	int y3 = rand() % LCD22_HEIGHT;
	int r0  = rand() % 50;
	int r1  = rand() % 50;
	int t0 = (rand() % 6) + 1;
	int t1 = (rand() % 6) + 1;

	leds_set_mask(mask_leds, keys_get_mask(mask_keys));

	if (keys_get(3)) {
		lcd22_draw_string_P(txt, 0, 0, LCD22_BLACK, LCD22_WHITE, kLCDFont8x16Regular);
		_delay_ms(1000);
	}

	lcd22_draw_line(x0, y0, x1, y1, t0, LCD22_RED);

	int m = rand() % 5;

	lcd22_draw_circle(x2, y2, r0, t1, LCD22_BLUE);
	if (m == 0)
		lcd22_draw_vgradient_rectangle(x2, y2, x3, y3, LCD22_RED, LCD22_YELLOW);
	else if (m == 1)
		lcd22_draw_hgradient_rectangle(x2, y2, x3, y3, LCD22_RED, LCD22_YELLOW);
	else if (m == 2)
		lcd22_draw_vgradient_circle(x3, y3, r1, LCD22_PURPLE, LCD22_CYAN);
	else if (m == 3)
		lcd22_draw_hgradient_circle(x3, y3, r1, LCD22_PURPLE, LCD22_CYAN);
	else {
		lcd22_draw_circle(x2, y2, r0, t1, LCD22_BLUE);
		lcd22_draw_filled_circle(x3, y3, r1, LCD22_BLUE);
	}

	_delay_ms(100);
}

#endif // TESTMETHOD == 0

#if TESTMETHOD == 1

void testTouch() {
	int16_t touchX1, touchY1, touchX2, touchY2;
	if (lcd22_get_touch(&touchX1, &touchY1)) {
		lcd22_draw_dot(touchX1, touchY1, 3, LCD22_BLACK);
		while (lcd22_get_touch(&touchX2, &touchY2)) {
			lcd22_draw_dot(touchX2, touchY2, 3, LCD22_BLACK);
		}
		int16_t radius = sqrt_integer_rounded(SQR(DIFF(touchX1, touchX2)) + SQR(DIFF(touchY1, touchY2)));
		lcd22_draw_line(touchX1, touchY1, touchX2, touchY2, 3, LCD22_RED);
		lcd22_draw_circle(touchX1, touchY1, radius, 3, LCD22_RED);
	}
}

#endif // TESTMETHOD == 1

#if TESTMETHOD == 2

void testTouchD() {
	char str[64];

	int16_t touchX, touchY;
	if (lcd22_get_touch(&touchX, &touchY))
		lcd22_draw_dot(touchX, touchY, 3, LCD22_BLACK);
}

#endif // TESTMETHOD == 2

#if TESTMETHOD == 3

int16_t posX = 0, posY = 0;
uint8_t curColor = 0;

static const uint16_t colors[] = {
	LCD22_BLACK,
	LCD22_CYAN,
	LCD22_PURPLE,
	LCD22_RED,
	LCD22_BLUE,
	LCD22_WHITE,
	LCD22_YELLOW,
	LCD22_ORANGE,
	LCD22_GRAY,
	LCD22_LGRAY,
	LCD22_DARKGRAY,
	LCD22_PORPO,
	LCD22_PINK,
	LCD22_GRAYBLUE,
	LCD22_LGRAYBLUE,
	LCD22_DARKBLUE,
	LCD22_LIGHTBLUE
};

void testJoystick() {
	lcd22_draw_dot(posX, posY, 3, colors[curColor]);
	_delay_ms(10);

	joystick_state_t joy = joystick_get();

	if (joy == kJoystickDown) {
		curColor = (curColor + 1) % ARRAYSIZE(colors);
		lcd22_draw_dot(posX, posY, 3, colors[curColor]);
		_delay_ms(300);
	}

	if (joy == kJoystickA)
		posY--;
	if (joy == kJoystickB)
		posX++;
	if (joy == kJoystickC)
		posX--;
	if (joy == kJoystickD)
		posY++;

	posX = CLIP(posX, 0, LCD22_WIDTH  - 1);
	posY = CLIP(posY, 0, LCD22_HEIGHT - 1);
}

#endif // TESTMETHOD == 3

#if TESTMETHOD == 4

void testKeypadAD() {
	uint8_t key = keypadad_get(0);

	char str[64];

	sprintf(str, "Pressed key: %d", key);
	pad_string(str, 32, ' ');
	lcd22_draw_string(str, 0, 0, LCD22_BLACK, LCD22_GREEN, kLCDFont8x16Regular);
}

#endif // TESTMETHOD == 4

#if TESTMETHOD == 5

void testKeypad4x4() {
	keypad4x4_key_t key = keypad4x4_get(&keypad4x4);

	char str[64];

	sprintf(str, "Pressed key: %d", key);
	pad_string(str, 32, ' ');
	lcd22_draw_string(str, 0, 0, LCD22_BLACK, LCD22_GREEN, kLCDFont8x16Regular);
}

#endif // TESTMETHOD == 5

#if TESTMETHOD == 6

void testKeypad5io() {
	keypad5io_key_t key = keypad5io_get(&keypad5io);

	char str[64];

	sprintf(str, "Pressed key: %d", key);
	pad_string(str, 32, ' ');
	lcd22_draw_string(str, 0, 0, LCD22_BLACK, LCD22_GREEN, kLCDFont8x16Regular);
}

#endif // TESTMETHOD == 6

#if TESTMETHOD == 7

void testDS18B20() {
	generic_io_t ds18b20 = generic_io_create(kPortG, 2);
	int16_t temp = ds18b20_get(&ds18b20); //readTemp();

	char str[64];

	sprintf(str, "Temperature: % 02d.%02dC", temp / 100, temp % 100);
	pad_string(str, 32, ' ');
	lcd22_draw_string(str, 0, 0, LCD22_BLACK, LCD22_GREEN, kLCDFont8x16Regular);
}

#endif // TESTMETHOD == 7

#if TESTMETHOD == 8

void testSegment() {

	uint16_t n = 0;
	uint8_t  s = 0;
	bool colon = FALSE;
	while (1) {
		segment_set_full_hex(&segment, s, s+1, s+2, s+3, colon);

		n = (n + 1) & 0x7FF;
		if (n == 0)
			s = (s + 1) % 16;
		if ((n & 0x3FF) == 0)
			colon = !colon;
	}
}

#endif // TESTMETHOD == 8

#if TESTMETHOD == 9

void testLCD22Font() {
	char str[128];

	for (int i = 0;  i < 127; i++)
		str[i] = i + 1;
	str[127] = 0;

	lcd22_draw_string(str, 0, 0, LCD22_BLACK, LCD22_WHITE, kLCDFont8x8Regular);
	lcd22_draw_string_P(txt, 0, 5*16, LCD22_BLACK, LCD22_WHITE, kLCDFont8x8Regular);
}

#endif // TESTMETHOD == 9

#if TESTMETHOD == 10

static const char *kWeekDays[7] = {"MO", "TU", "WE", "TH", "FR", "SA", "SU"};

void testLCD12864ST(void) {
	static int16_t OLD_temp = 0xFFF;
	static pcf8563_time_t OLD_time = {FALSE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFFFF};
	static keypadtouch_state_t OLD_touch = { TRUE, TRUE, TRUE };

	generic_io_t ds18b20 = generic_io_create(kPortG, 2);
	int16_t temp = ds18b20_get(&ds18b20);

	pcf8563_time_t time;
	bool time_res = pcf8563_get(&time);

	keypadtouch_state_t touch = keypadtouch_get(&keypadtouch);

	if (memcmp(&time, &OLD_time, sizeof(time))) {
		if (time_res) {
			lcd12864st_printf(0, 0, "%04d-%02d-%02d (%s)", time.year, time.month, time.day, kWeekDays[time.weekday]);
			lcd12864st_printf(0, 1, "%02d:%02d:%02d", time.hour, time.minute, time.second);
			lcd12864st_printf(14, 1, "%d", valid_start_time);

			if (!time.integrity) {
				lcd12864st_print(15, 0, "!");
				lcd12864st_print(15, 1, "!");
			}
		} else {
			lcd12864st_printf(0, 0, "---------- (--)");
			lcd12864st_printf(0, 1, "--:--:--");
		}

		lcd12864st_refreshLine(0);
		lcd12864st_refreshLine(1);

		lcd12864st_refresh();

		memcpy(&OLD_time, &time, sizeof(time));
	}

	if (OLD_temp != temp) {
		lcd12864st_printf(0, 3, "%02d.%02dC", temp / 100, temp % 100);
		lcd12864st_refreshLine(3);

		OLD_temp = temp;
	}

	if (memcmp(&touch, &OLD_touch, sizeof(touch))) {
		lcd12864st_clearLine(2);
		lcd12864st_printf(0, 2, "%1d %1d %1d %1d", touch.key1, touch.key2, touch.key3, touch.slider);
		lcd12864st_refreshLine(2);

		memcpy(&OLD_touch, &touch, sizeof(touch));
	}
}

#endif // TESTMETHOD == 10

#if TESTMETHOD == 11

#define AT24A (0xA2 >> 1)

uint8_t data = 0x23;
void testAT24() {
	DDRA = 0xFF;
	PORTA = 0;
	while (1) {
		uint8_t rData = 0x00;

		lcd12864st_clear();
		PORTA = 1;

		bool writeRes = i2c_write_byte(AT24A, 0x02, data);
		PORTA = 2;
		while (!i2c_poll(AT24A, TRUE));
		PORTA = 3;
		bool writeRes2 = i2c_write_byte(AT24A, 0x03, data);
		PORTA = 4;
		lcd12864st_printf(0, 0, "Write 0x%02X: %d %d", data, writeRes, writeRes2);
		PORTA = 5;
		while (!i2c_poll(AT24A, FALSE));
		PORTA = 6;
		bool readRes = i2c_read(AT24A, 0x02, 1, &rData);
		PORTA = 7;
		lcd12864st_printf(0, 1, "Read 0x%02X: %d", rData, readRes);

		lcd12864st_refresh();
		_delay_ms(1000);
	}
}

#endif // TESTMETHOD == 11

#if TESTMETHOD == 12

void testMAG3110() {
	usart0_init();
	printf("\n\n---\n");

	bool initRes = mag3110_init();
	printf("Inited: %d\n", initRes);

	while (1) {
		if (!mag3110_wait_measurement())
			continue;

		int8_t temp = 0x7F;
		mag3110_get_temperature(&temp);

		int16_t x, y, z;
		if (mag3110_get(&x, &y, &z))
			printf("Measurement: %d, %d, %d (%d C)\n", x, y, z, temp);
	}
}

#endif // TESTMETHOD == 12

#if TESTMETHOD == 13

pcf8574_t pcf8574;
void testPCF8574() {
	usart0_init();
	printf("\n\n---\n");

	printf("Init: %d\n", pcf8574_init(&pcf8574, kPCF8574, 0, generic_io_create(kPortNone, 0)));

	uint8_t n = 0;
	while (1) {
		printf("Set %d: %d\n", n, pcf8574_set(&pcf8574, n));
		n++;
		_delay_ms(100);
	}
}

#endif // TESTMETHOD == 13

#if TESTMETHOD == 14

void testPCF8591() {
	usart0_init();
	printf("\n\n---\n");

	printf("Init: %d\n", pcf8591_init(0));
	uint8_t n = 0;
	bool r = TRUE;
	while (1) {
		if (!pcf8591_set(0, n))
			continue;

		if (r) {
			if (++n == 255)
				r = !r;
		} else {
			if (--n == 0)
				r = !r;
		}

		uint8_t data[4];
		if (pcf8591_get_all(0, kPCF8591InputSingle, data))
			printf("-> %3d, %3d, %3d, %3d\n", data[0], data[1], data[2], data[3]);
	}
}

#endif // TESTMETHOD == 14

#if TESTMETHOD == 15

void print_blocks_bad(k9fxx08x0c_t *k9fxx08x0c) {
	printf(".--- Bad block ---.\n");
	for (uint16_t i = 0; i < k9fxx08x0c->block_count; i++)
		if (k9fxx08x0c_is_block_bad(k9fxx08x0c, i))
			printf("| %d\n", i);
	printf("'-----------------'\n");
}

bool erase_block(k9fxx08x0c_t *k9fxx08x0c, uint8_t block) {
	bool success = k9fxx08x0c_erase_block(k9fxx08x0c, block);

	printf("Erased block %d: %d\n", block, success);

	return success;
}

void print_page(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page) {
	uint8_t data[2048];

	printf("/--- Block %3d, page %2d (%d) ---\\\n", block, page, k9fxx08x0c_is_block_bad(k9fxx08x0c, block));
	if (k9fxx08x0c_read_page(k9fxx08x0c, block, page, data)) {
		usart0_dump_hex(data, sizeof(data), 0);
		printf("--> VERIFY: %d\n", k9fxx08x0c_verify_page(k9fxx08x0c, block, page, data, FALSE));
	} else
		printf("FAIL\n");
	printf("\\------------------------------/\n");
}

bool write_page(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page) {
	uint8_t data[2048];

	for (uint16_t i = 0; i < sizeof(data); i += 64)
		memcpy(data + i, "Lorem ipsum dolor sit amet durtum, consectetur adipiscing elit. ", 64);

	if (!erase_block(k9fxx08x0c, block))
		return FALSE;

	bool success;
	success = k9fxx08x0c_write_page(k9fxx08x0c, block, page, data);
	printf("Written page %d.%d: %d\n", block, page, success);
	if (!success)
		return FALSE;

	success = k9fxx08x0c_verify_page(k9fxx08x0c, block, page, data, FALSE);
	printf("Verify page %d.%d: %d\n", block, page, success);

	return success;
}

bool flash_from_usart0(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint32_t size) {
	uint32_t block_size = (uint32_t)k9fxx08x0c->pages_per_block * k9fxx08x0c->page_size;

	uint16_t blocks = size / block_size;
	if (size % block_size)
		blocks++;

	lcd12864st_clear();
	lcd12864st_printf(0, 0, "%lu b",  size);
	lcd12864st_printf(0, 1, "%u %u", blocks, block);
	lcd12864st_refresh();

	for (uint32_t i = 0; i < blocks; i++) {
		lcd12864st_clearLine(2);
		lcd12864st_printf(0, 2, "E %d", block + i);
		lcd12864st_refresh();
		if (!k9fxx08x0c_erase_block(k9fxx08x0c, block + i))
			return FALSE;
	}


	uint8_t data[2048];
	uint16_t page = 0;

	while (size > 0) {
		uint16_t n = MIN(size, 2048);

		lcd12864st_clearLine(2);
		lcd12864st_printf(0, 2, "R %d.%d", block, page);
		lcd12864st_refresh();

		printf("ACK\n");
		uint16_t r = usart0_read(data, n);
		if (r != n) {
			lcd12864st_clearLine(2);
			lcd12864st_printf(0, 2, "RF %d.%d %d %02X", block, page, r, 0);
			lcd12864st_refresh();
			return FALSE;
		}

		lcd12864st_clearLine(2);
		lcd12864st_printf(0, 2, "W %d.%d", block, page);
		lcd12864st_refresh();

		memset(data + n, 0xFF, 2048 - n);
		if (!k9fxx08x0c_write_page(k9fxx08x0c, block, page, data))
			return FALSE;
		if (!k9fxx08x0c_verify_page(k9fxx08x0c, block, page, data, FALSE))
			return FALSE;

		size -= n;
		page = (page + 1) % k9fxx08x0c->pages_per_block;
		if (page == 0)
			block++;
	}
	printf("ACK\n");

	return TRUE;
}

bool dump_data(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page, uint16_t offset, uint32_t size) {
	bool first = TRUE;

	uint8_t data[2048];
	while (size > 0) {
		if (first)
			printf(".--- Block %4d, page %2d ---\n", block, page);
		else
			printf("|--- Block %4d, page %2d ---\n", block, page);
		first = FALSE;

		uint32_t address = k9fxx08x0c_address(k9fxx08x0c, block, page, offset);
		uint16_t n = MIN(size, k9fxx08x0c->page_size - offset);

		k9fxx08x0c_read_page(k9fxx08x0c, block, page, data);

		usart0_dump_hex(data + offset, n, address);

		size  -= n;
		offset = 0;

		page = (page + 1) % k9fxx08x0c->pages_per_block;
		if (page == 0)
			block = (block + 1) % k9fxx08x0c->block_count;

		if (size == 0)
			printf("'---------------------------\n");
	}

	return TRUE;
}

bool display_success(bool success) {
	lcd12864st_clearLine(3);
	lcd12864st_printf(0, 3, success ? "SUCCESS": "FAILURE");
	lcd12864st_refresh();

	return success;
}

void erase_all(k9fxx08x0c_t *k9fxx08x0c) {
	printf(".--- Erasing everything:\n");
	for (uint16_t i = 0; i < k9fxx08x0c->block_count; i++) {
		if (!k9fxx08x0c_erase_block(k9fxx08x0c, i))
			printf("FAIL %d\n", i);
	}
	printf("'---\n");
}

void test_erase(k9fxx08x0c_t *k9fxx08x0c) {
	uint16_t blockStart = 0;
	const int blocks = 10;

	uint8_t data[2048];

	printf("---- Erase:\n");
	for (int i = 0; i < blocks; i++) {
		k9fxx08x0c_erase_block(k9fxx08x0c, blockStart + i);
	}

	printf("---- Write:\n");
	for (int i = 0; i < blocks; i++) {
		memset(data, i, sizeof(data));

		k9fxx08x0c_write_page(k9fxx08x0c, blockStart + i, 0, data);
	}

	printf("---- Read:\n");
	for (int i = 0; i < blocks; i++) {

		memset(data, 0xFF, sizeof(data));
		k9fxx08x0c_read_page(k9fxx08x0c, blockStart + i, 0, data);
		usart0_dump_hex(data, 16, k9fxx08x0c_address(k9fxx08x0c, blockStart + i, 0, 0));
	}

	const int toErase = 1;
	printf("---- Erase %d + read:\n", blockStart + toErase);
	k9fxx08x0c_erase_block(k9fxx08x0c, blockStart + toErase);
	for (int i = 0; i < blocks; i++) {
		memset(data, 0xFF, sizeof(data));
		k9fxx08x0c_read_page(k9fxx08x0c, blockStart + i, 0, data);
		usart0_dump_hex(data, 16, k9fxx08x0c_address(k9fxx08x0c, blockStart + i, 0, 0));
	}
}

void print_info(k9fxx08x0c_t *k9fxx08x0c) {
	printf(".---\n");
	printf("| Page size: %d (%d)\n", k9fxx08x0c->page_size, k9fxx08x0c->page_full_size);
	printf("| Pages per block: %d\n", k9fxx08x0c->pages_per_block);
	printf("| Blocks: %d\n", k9fxx08x0c->block_count);
	printf("| Block size: %lu (%lu)\n", (long int) k9fxx08x0c->pages_per_block * k9fxx08x0c->page_size, (long int) k9fxx08x0c->pages_per_block * k9fxx08x0c->page_full_size);
	printf("| WP: %d\n", k9fxx08x0c_is_write_protected(k9fxx08x0c));
	printf("'---\n");
}

void testNAND() {
	lcd12864st_init();
	usart0_init();
	printf("\n\n---\n");

	lcd12864st_print(0, 0, "Waiting...");
	lcd12864st_refresh();

	k9fxx08x0c_t k9fxx08x0c;
	if (!k9fxx08x0c_init(&k9fxx08x0c) || (k9fxx08x0c.page_size != 2048))
		goto FAIL;

	print_info(&k9fxx08x0c);
	print_blocks_bad(&k9fxx08x0c);

	char cmd[32];
	while (1) {
		uint32_t size;
		uint16_t block, page, offset;

		memset(cmd, 0, sizeof(cmd));
		fgets(cmd, sizeof(cmd), stdin);
		if        (sscanf(cmd, "flash %lu %u", &size, &block) == 2) {
			if (display_success(flash_from_usart0(&k9fxx08x0c, block, size)))
				print_page(&k9fxx08x0c, block, 0);
			else
				printf("NACK\n");
		} else if (sscanf(cmd, "dump %u %u %u %lu", &block, &page, &offset, &size) == 4) {
			display_success(dump_data(&k9fxx08x0c, block, page, offset, size));
		} else if (!strncmp(cmd, "teste", 5)) {
			test_erase(&k9fxx08x0c);
		} else if (!strncmp(cmd, "eraseall", 8)) {
			erase_all(&k9fxx08x0c);
		} else if (!strncmp(cmd, "info", 4)) {
			print_info(&k9fxx08x0c);
		} else if (!strncmp(cmd, "badblocks", 9)) {
			print_blocks_bad(&k9fxx08x0c);
		}
	}

FAIL:
	printf("FAIL\n");
	while (1);
}

#endif // TESTMETHOD == 15

#if TESTMETHOD == 16

typedef struct {
	const char *name;
	uint16_t block;
	uint16_t page;
	uint32_t size;
} song_t;

static const song_t kSongs[] = {
	{ "drugula.mp3" ,  0, 0, 5854353 },
	{ "friedhof.mid", 45, 0,   25115 },
	{ "jones.mid"   , 46, 0,   31643 }
};

typedef struct {
	const song_t *song;

	uint16_t block;
	uint16_t page;
	uint16_t offset;
	uint32_t size;
} play_state_t;

static play_state_t play_state = { 0, 0, 0, 0, 0 };

void list_songs(void) {
	printf(".-- ID --.---- Name ----.--- Size ---.- Block -.- Page -.\n");
	for (uint16_t i = 0; i < ARRAYSIZE(kSongs); i++)
		printf("| %6u | %12s | %10lu | %7u | %6u |\n", i, kSongs[i].name, kSongs[i].size, kSongs[i].block, kSongs[i].page);
	printf("'--------'--------------'------------'---------'--------'\n");
}

bool play_is_playing(void) {
	return play_state.song != 0;
}

bool play_stop(void) {
	if (!play_state.song)
		return TRUE;

	vs1003b_stop();
	play_state.song = 0;
	return TRUE;
}

bool play_start(const song_t *song) {
	play_stop();

	if (!song)
		return FALSE;

	play_state.song = song;

	play_state.block  = song->block;
	play_state.page   = song->page;
	play_state.offset = 0;
	play_state.size   = song->size;

	return TRUE;
}

#define PLAY_FEED_BUFFER 128
bool play_feed_data(k9fxx08x0c_t *k9fxx08x0c) {
	if (!play_state.song)
		return TRUE;

	uint16_t fed = 0xFFFF;
	while ((play_state.size > 0) && (fed > 0) && vs1003b_ready()) {
		uint8_t data[PLAY_FEED_BUFFER];

		uint16_t count = MIN(MIN(play_state.size, sizeof(data)), k9fxx08x0c->page_size - play_state.offset);
		if (!k9fxx08x0c_read_page_part(k9fxx08x0c, play_state.block, play_state.page, play_state.offset, count, data))
			return FALSE;

		fed = vs1003b_feed_data(data, count);

		play_state.size -= fed;

		uint16_t offset = (play_state.offset + fed) % k9fxx08x0c->page_size;
		if (offset < play_state.offset) {
			play_state.page = (play_state.page + 1) % k9fxx08x0c->pages_per_block;
			if (play_state.page == 0)
				play_state.block = (play_state.block + 1) % k9fxx08x0c->block_count;
		}

		play_state.offset = offset;
	}

	return TRUE;
}

bool play_song_by_id(uint16_t id) {
	if (id >= ARRAYSIZE(kSongs))
		return FALSE;

	return play_start(&kSongs[id]);
}

bool play_song_by_name(const char *name) {
	for (uint16_t i = 0; i < ARRAYSIZE(kSongs); i++)
		if (!strcmp(kSongs[i].name, name))
			return play_song_by_id(i);

	return FALSE;
}

void play_get_volume(void) {
	uint8_t left, right;
	vs1003b_get_volume(&left, &right);

	printf("%u %u\n", left, right);
}

void play_set_volume(uint8_t left, uint8_t right) {
	vs1003b_set_volume(left, right);
}

void play_display_status(void) {
	printf("Playing: %s. Format: %u. Time: %u\n", play_is_playing() ? "yes" : "no", (unsigned int) vs1003b_get_format(), vs1003b_get_decode_time());
}


bool read_input(char *command, uint16_t size) {
	int c = usart0_get();
	while (c >= 0) {
		if (c == '\r')
			return TRUE;
		if (c == '\n')
			return TRUE;

		uint16_t n = strlen(command);
		if (n >= (size - 1)) {
			command[0] = '\0';
			n = 0;
		}

		command[n]     = c;
		command[n + 1] = '\0';

		c = usart0_get();
	}

	return FALSE;
}

bool execute_command(char *command) {
	if (command[0] == '\0')
		return TRUE;

	if (!strcmp(command, "help")) {
		printf("Valid commands are:\n");
		printf("  list\n");
		printf("  play <songname>\n");
		printf("  playid <songid>\n");
		printf("  stop\n");
		printf("  getvol\n");
		printf("  setvol <left> <right>\n");
		printf("  status\n");
		return TRUE;
	}

	if (!strcmp(command, "list")) {
		list_songs();
		return TRUE;
	}

	if (!strcmp(command, "stop")) {
		play_stop();
		return TRUE;
	}

	if (!strncmp(command, "play ", 5)) {
		bool result = play_song_by_name(command + 5);
		if (!result)
			printf("Failed to play \"%s\"\n", command + 5);
		return result;
	}

	unsigned int id;
	if (sscanf(command, "playid %u", &id) == 1) {
		bool result = play_song_by_id(id);
		if (!result)
			printf("Failed to play %d\n", id);
		return result;
	}

	if (!strcmp(command, "getvol")) {
		play_get_volume();
		return TRUE;
	}

	unsigned int left, right;
	if (sscanf(command, "setvol %u %u", &left, &right) == 2) {
		play_set_volume(left, right);
		return TRUE;
	}

	if (!strcmp(command, "status")) {
		play_display_status();
		return TRUE;
	}

	printf("Invalid command \"%s\"\n", command);
	return FALSE;
}

void testMP3() {
	usart0_init();
	printf("\n\n---\n");

	if (!vs1003b_init())
		goto FAIL;

	k9fxx08x0c_t k9fxx08x0c;
	if (!k9fxx08x0c_init(&k9fxx08x0c) || (k9fxx08x0c.page_size != 2048))
		goto FAIL;

	vs1003b_set_volume(175, 175);
	vs1003b_set_bass_treble(15, 8, 0, 0);

	char command[128];
	command[0] = '\0';

	while (1) {
		if (read_input(command, sizeof(command))) {
			execute_command(command);
			command[0] = '\0';
		}

		if (!play_feed_data(&k9fxx08x0c)) {
			printf("Failed feed data\n");
			goto FAIL;
		}
	}

FAIL:
	printf("FAIL\n");
	while (1);
}

#endif // TESTMETHOD == 16
