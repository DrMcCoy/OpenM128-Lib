/* VS1003B - Using the VS1003B MP3/WMA/WAV/MID decoder
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

#include <avr/io.h>
#include <util/delay.h>

#include "openm128/vs1003b.h"
#include "openm128/util.h"

// Commands
#define COMMAND_MODE       0x00
#define COMMAND_STATUS     0x01
#define COMMAND_BASS       0x02
#define COMMAND_CLOCKF     0x03
#define COMMAND_DECODETIME 0x04
#define COMMAND_AUDATA     0x05
#define COMMAND_HEADER0    0x08
#define COMMAND_HEADER1    0x09
#define COMMAND_VOLUME     0x0B

// Remember the current volume and bass/treble setting
static uint16_t vs1003b_current_volume = 0xFEFE;
static uint16_t vs1003b_current_bass = 0x0000;


// -- Low level bit fiddling --

static void vs1003b_set_cs(void) {
	PORTB &= ~(1 << PB0);
}

static void vs1003b_set_dcs(void) {
	PORTE &= ~(1 << PE6);
}

static void vs1003b_clear_cs(void) {
	PORTB |= 1 << PB0;
}

static void vs1003b_clear_dcs(void) {
	PORTE |= 1 << PE6;
}

static void vs1003b_spi_send(uint8_t data) {
	SPDR = data;
	while (!(SPSR & (1 << SPIF)));
}

static uint8_t vs1003b_spi_receive(void) {
	SPDR = 0x00;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}

static bool vs1003b_is_busy(void) {
	return !(PIND & (1 << PD6));
}

static void vs1003b_hardware_reset(void) {
	PORTD &= ~(1 << PD7);
	_delay_us(1);
	PORTD |=   1 << PD7;
}


// -- Internal command and data wrangling --

static void vs1003b_wait_while_busy(void) {
	while (vs1003b_is_busy());
}

static void vs1003b_command_write(uint8_t reg, uint16_t data) {
	vs1003b_set_cs();

	vs1003b_spi_send(0x02);

	vs1003b_spi_send(reg);

	vs1003b_spi_send((data & 0xFF00) >> 8);
	vs1003b_spi_send( data & 0x00FF);

	vs1003b_clear_cs();
}

static uint16_t vs1003b_command_read(uint8_t reg) {
	vs1003b_set_cs();

	vs1003b_spi_send(0x03);

	vs1003b_spi_send(reg);

	uint8_t high = vs1003b_spi_receive();
	uint8_t low  = vs1003b_spi_receive();

	vs1003b_clear_cs();

	return (((uint16_t) high) << 8) | low;
}

static void vs1003b_command_bass(uint16_t control) {
	vs1003b_current_bass = control;

	vs1003b_command_write(COMMAND_BASS, control);
	vs1003b_wait_while_busy();
}

static void vs1003b_command_volume(uint16_t volume) {
	vs1003b_current_volume = volume;

	vs1003b_command_write(COMMAND_VOLUME, volume);
	vs1003b_wait_while_busy();
}

static void vs1003b_data_write(uint8_t *data, uint8_t n) {
	vs1003b_set_dcs();

	while (n-- > 0)
		vs1003b_spi_send(*data++);

	vs1003b_clear_dcs();
}

static void vs1003b_data_empty() {
	if (vs1003b_is_busy())
		return;

	vs1003b_set_dcs();

	for (int i = 0; i < 32; i++)
		vs1003b_spi_send(0x00);

	vs1003b_clear_dcs();
}

static void vs1003b_init_registers(void) {
	vs1003b_wait_while_busy();

	/* Set a clock multiplier of 4.5x.
	   This is the highest supported clock and will allow the most simultaneous features.
	   It will also draw the most power.
	 */
	vs1003b_command_write(COMMAND_CLOCKF, 0xE000);
	vs1003b_wait_while_busy();

	vs1003b_command_volume(vs1003b_current_volume);
	vs1003b_command_bass(vs1003b_current_bass);
}


// -- Public VS1003B functions --

bool vs1003b_init(void) {
	// Set correct pin directions
	DDRB  = (DDRB & ~(1 << PB3)) | (1 << PB0) | (1 << PB1) | (1 << PB2);
	DDRD  = (DDRD & ~(1 << PD6)) | (1 << PD7);
	DDRE |= 1 << PE6;

	// Set pull-ups and drive pins into default levels
	PORTB  = (PORTB & ~((1 << PB1) | (1 << PB2))) | (1 << PB0) | (1 << PB3);
	PORTD |= (1 << PD6) | (1 << PD7);
	PORTE |= (1 << PE6);

	SPCR = 0x50; // Enable SPI + SPI master (no interrupt, MSB first, leading rising edge, fosc/4 clock)
	SPSR = 0x00; // Clear SPI status register, normal SPI speed

	vs1003b_reset();

	// Make sure we're really using a VS1003B
	uint16_t mode   = vs1003b_command_read(COMMAND_MODE);
	uint16_t status = vs1003b_command_read(COMMAND_STATUS);
	if (((status & 0x0030) != 0x0030) || !(mode & 0x0800))
		return FALSE;

	return TRUE;
}

void vs1003b_reset(void) {
	vs1003b_hardware_reset();
	vs1003b_init_registers();
}

bool vs1003b_ready(void) {
	return !vs1003b_is_busy();
}

uint16_t vs1003b_feed_data(uint8_t *data, uint16_t count) {
	uint16_t processed = 0;
	while (count > 0) {
		if (!vs1003b_ready())
			break;

		uint8_t n = MIN(count, 32);

		vs1003b_data_write(data, n);

		processed += n;
		count     -= n;
		data      += n;
	}

	return processed;
}

void vs1003b_stop(void) {
	// Mute
	vs1003b_command_write(COMMAND_VOLUME, 0xFEFE);

	// Send stop command and write 0x00 until the format field has been cleared (= no current sound). MP3 can be stopped immediately.
	vs1003b_command_write(COMMAND_MODE, 0x8008);
	vs1003b_format_t format = vs1003b_get_format();
	while ((format != kVS1003BFormatNone) && (format != kVS1003BFormatMP3)) {
		vs1003b_data_empty();
		format = vs1003b_get_format();
	}

	// Reset the VS1003B, just to be sure a broken format didn't mess anything up
	vs1003b_reset();
}

void vs1003b_set_volume(uint8_t left, uint8_t right) {
	// 0 is full volume, 254 is silence
	left  = 254 - MIN(left , 254);
	right = 254 - MIN(right, 254);

	vs1003b_command_volume(((uint16_t)left << 8) | right);
}

uint16_t vs1003b_get_decode_time(void) {
	return vs1003b_command_read(COMMAND_DECODETIME);
}

vs1003b_format_t vs1003b_get_format(void) {
	uint16_t header1 = vs1003b_command_read(COMMAND_HEADER1);

	if (header1 == 0x0000)
		return kVS1003BFormatNone;
	if (header1 == 0x7665)
		return kVS1003BFormatWAV;
	if (header1 == 0x574D)
		return kVS1003BFormatWMA;
	if (header1 == 0x4D54)
		return kVS1003BFormatMID;

	if ((header1 >> 5) == 0x7FF)
		return kVS1003BFormatMP3;

	return kVS1003BFormatUnknown;
}

bool vs1003b_set_bass_treble(uint8_t bass_amplitude, uint8_t bass_freqlimit, int8_t treble_amplitude, uint8_t treble_freqlimit) {
	if (bass_amplitude > 15)
		return FALSE;
	if ((bass_freqlimit < 20) || (bass_freqlimit > 150))
		return FALSE;
	if ((treble_amplitude < -8) || (treble_amplitude > 7))
		return FALSE;
	if (treble_freqlimit > 15)
		return FALSE;

	bass_freqlimit /= 10;

	uint16_t control = (((uint16_t)((uint8_t)treble_amplitude)) << 12) | (((uint16_t)treble_freqlimit) << 8) | (bass_amplitude << 4) | (bass_freqlimit);

	vs1003b_command_bass(control);
	return TRUE;
}
