/* usart0 - Using the USART0 on the OpenM128 board
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

#include "openm128/usart0.h"
#include "openm128/util.h"

static bool usart0_echo_enabled = FALSE;

void usart0_put(unsigned char c) {
	while(!(UCSR0A & 0x20));
	UDR0 = c;
}

int usart0_get(void) {
	if(!(UCSR0A & 0x80))
		return _FDEV_EOF;

	uint8_t status = UCSR0A;
	uint8_t data   = UDR0;

	if (status & 0x1C)
		return _FDEV_ERR;

	if (usart0_echo_enabled) {
		usart0_put(data);
		if (data == '\r')
			usart0_put('\n');
	}

	return data;
}

uint16_t usart0_read(uint8_t *data, uint16_t n) {
	uint16_t processed = 0;
	while (n-- > 0) {
		while (!(UCSR0A & 0x80));

		uint8_t status = UCSR0A;
		*data++        = UDR0;

		if (status & 0x1C)
			break;

		processed++;
	}

	return processed;
}

uint16_t usart0_write(uint8_t *data, uint16_t n) {
	uint16_t processed = 0;
	while (n-- > 0) {
		while(!(UCSR0A & 0x20));

		UDR0 = *data++;

		processed++;
	}

	return processed;
}

int usart0_get_wait(void) {
	int c;
	while ((c = usart0_get()) == _FDEV_EOF);

	return c;
}


static int uart0_putchar(char c, FILE *stream) {
	if (c == '\n')
		uart0_putchar('\r', stream);

	usart0_put(c);
	return 0;
}

static int uart0_getchar(FILE *stream) {
	int data = _FDEV_EOF;
	while (data == _FDEV_EOF)
		data = usart0_get();

	if (data == '\r')
		data = '\n';

	return data;
}

static FILE uart0_stdout_stdin = FDEV_SETUP_STREAM(uart0_putchar, uart0_getchar, _FDEV_SETUP_RW);

void usart0_init(void) {
	// USART0 initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART0 Receiver: On
	// USART0 Transmitter: On
	// USART0 Mode: Asynchronous
	// USART0 Baud Rate: 9600
	UCSR0A = 0x00;
	UCSR0B = 0x98;
	UCSR0C = 0x06;
	UBRR0H = 0x00;
	UBRR0L = 0x2F;

	stdout = &uart0_stdout_stdin;
	stdin  = &uart0_stdout_stdin;
}

void usart0_echo(bool enabled) {
	usart0_echo_enabled = enabled;
}

void usart0_dump_hex(const uint8_t *data, uint16_t size, uint32_t address) {
	if (size == 0)
		return;

	while (size > 0) {
		// At max 16 bytes printed per row
		uint16_t n = MIN(size, 16);

		// Print an address
		printf("%04X%04X  ", (unsigned int)(address >> 16), (unsigned int)(address & 0xFFFF));

		// 2 "blobs" of each 8 bytes per row
		for (uint8_t i = 0; i < 2; i++) {
			for (uint8_t j = 0; j < 8; j++) {
				uint8_t m = i * 8 + j;

				if (m < n)
					// Print the data
					printf("%02X ", data[m]);
				else
					// Last row, data count not aligned to 16
					printf("   ");
			}

			// Separate the blobs by an extra space
			printf(" ");
		}

		printf("|");

		// If the data byte is a printable character, print it. If not, substitute a '.'
		for (uint8_t i = 0; i < n; i++)
			printf("%c", (data[i] > 31 && data[i] < 127) ? data[i] : '.');

		printf("|\n");

		size   -= n;
		address += n;
		data   += n;
	}
}
