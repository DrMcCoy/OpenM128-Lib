/* k9fxx08x0c - Reading from / writing to the K9FXX08X0C NAND flash memory
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

#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

#include "openm128/k9fxx08x0c.h"
#include "openm128/usart0.h"

// -- Internal defines --

#define K9FXX08X0C_MAKER_CODE  0xEC
#define K9FXX08X0C_DEVICE_CODE 0xF1

#define K9FXX08X0C_COMMAND_READ_ID       0x90
#define K9FXX08X0C_COMMAND_READ_STATUS   0x70
#define K9FXX08X0C_COMMAND_READ_ADDRESS  0x00
#define K9FXX08X0C_COMMAND_READ_START    0x30
#define K9FXX08X0C_COMMAND_ERASE_ADDRESS 0x60
#define K9FXX08X0C_COMMAND_ERASE_START   0xD0
#define K9FXX08X0C_COMMAND_WRITE_ADDRESS 0x80
#define K9FXX08X0C_COMMAND_WRITE_START   0x10

/** To reduce reading disturbance and wear, all bytes in a block should be read about equally often and frequent.
    Unfortunately, this messes with having to check for bad blocks, so we read all bad block markers at initialization
    and keep the information in RAM. Of course, the more blocks there are, the more storage we need for book-keeping.
 */
#define K9FXX08X0C_BLOCK_COUNT      1024
#define K9FXX08X0C_BAD_BLOCK_STORE (K9FXX08X0C_BLOCK_COUNT / 3)


// -- Forward declarations --

static bool k9fxx08x0c_write_page_part(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page,
                                       uint16_t offset, uint16_t n, uint8_t *data);


// -- Low-level I/O fiddling --

static bool k9fxx08x0c_is_ready() {
	return !!(PINC & (1 << PC3));
}

static void k9fxx08x0c_set_chip_enable() {
	PORTC &= ~(1 << PC0);
}

static void k9fxx08x0c_set_command() {
	PORTC |= (1 << PC5);
}

static void k9fxx08x0c_set_address() {
	PORTC |= (1 << PC4);
}

static void k9fxx08x0c_set_read() {
	PORTC &= ~(1 << PC1);
}

static void k9fxx08x0c_set_write() {
	PORTC &= ~(1 << PC2);
}

static void k9fxx08x0c_clear_chip_enable() {
	PORTC |= (1 << PC0);
}

static void k9fxx08x0c_clear_command() {
	PORTC &= ~(1 << PC5);
}

static void k9fxx08x0c_clear_address() {
	PORTC &= ~(1 << PC4);
}

static void k9fxx08x0c_clear_read() {
	PORTC |= (1 << PC1);
}

static void k9fxx08x0c_clear_write() {
	PORTC |= (1 << PC2);
}


// -- Getting/Setting data --

static void k9fxx08x0c_make_input() {
	DDRA  = 0x00;
	PORTA = 0xFF;

	_delay_us(1);
}

static void k9fxx08x0c_make_output() {
	DDRA  = 0xFF;
}

static void k9fxx08x0c_set_data(uint8_t data) {
	k9fxx08x0c_make_output();
	PORTA = data;
}

static uint8_t k9fxx08x0c_get_data() {
	k9fxx08x0c_make_input();
	return PINA;
}


// -- Sending/receiving data --

static void k9fxx08x0c_send_byte(uint8_t data) {
	k9fxx08x0c_set_write();
	k9fxx08x0c_set_data(data);
	k9fxx08x0c_clear_write();
}

static void k9fxx08x0c_send_data(uint16_t n, uint8_t *data) {
	while (n-- > 0)
		k9fxx08x0c_send_byte(*data++);
}

static uint8_t k9fxx08x0c_receive_byte() {
	k9fxx08x0c_set_read();
	uint8_t data = k9fxx08x0c_get_data();
	k9fxx08x0c_clear_read();

	return data;
}

static void k9fxx08x0c_receive_data(uint16_t n, uint8_t *data) {
	while (n-- > 0)
		*data++ = k9fxx08x0c_receive_byte();
}

static bool k9fxx08x0c_verify_data(uint16_t n, uint8_t *data) {
	while (n-- > 0)
		if (*data++ != k9fxx08x0c_receive_byte())
			return FALSE;

	return TRUE;
}


// -- Sending commands and addresses --

static void k9fxx08x0c_send_command(uint8_t command) {
	k9fxx08x0c_set_command();
	k9fxx08x0c_send_byte(command);
	k9fxx08x0c_clear_command();
}

static void k9fxx08x0c_send_address_byte(uint8_t address) {
	k9fxx08x0c_set_address();
	k9fxx08x0c_send_byte(address);
	k9fxx08x0c_clear_address();
}

/*
static void k9fxx08x0c_send_address_column(uint32_t address) {
	k9fxx08x0c_set_address();
	k9fxx08x0c_send_byte (address & 0x000000FFUL);
	k9fxx08x0c_send_byte((address & 0x00000F00UL) >> 8);
	k9fxx08x0c_clear_address();
}
*/

static void k9fxx08x0c_send_address_row(uint32_t address) {
	k9fxx08x0c_set_address();
	k9fxx08x0c_send_byte((address & 0x000FF000UL) >> 12);
	k9fxx08x0c_send_byte((address & 0x0FF00000UL) >> 20);
	k9fxx08x0c_clear_address();
}

static void k9fxx08x0c_send_address_full(uint32_t address) {
	k9fxx08x0c_set_address();
	k9fxx08x0c_send_byte (address & 0x000000FFUL);
	k9fxx08x0c_send_byte((address & 0x00000F00UL) >> 8 );
	k9fxx08x0c_send_byte((address & 0x000FF000UL) >> 12);
	k9fxx08x0c_send_byte((address & 0x0FF00000UL) >> 20);
	k9fxx08x0c_clear_address();
}


// -- Reading ID and status --

static void k9fxx08x0c_busy_wait() {
	_delay_us(1);
	while (!k9fxx08x0c_is_ready());
}

static bool k9fxx08x0c_read_id(uint8_t *data) {
	k9fxx08x0c_busy_wait();

	k9fxx08x0c_set_chip_enable();

	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_READ_ID);
	k9fxx08x0c_send_address_byte(0x00);

	k9fxx08x0c_receive_data(5, data);

	k9fxx08x0c_clear_chip_enable();

	return TRUE;
}

static uint8_t k9fxx08x0c_read_status() {
	k9fxx08x0c_set_chip_enable();

	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_READ_STATUS);

	uint8_t status = k9fxx08x0c_receive_byte();

	k9fxx08x0c_clear_chip_enable();

	return status;
}

static bool k9fxx08x0c_status_success() {
	return !(k9fxx08x0c_read_status() & 0x01);
}


// -- Bad block management --

static uint8_t k9fxx08x0c_bad_blocks[K9FXX08X0C_BAD_BLOCK_STORE];

static void k9fxx08x0c_bad_block_set(uint16_t block) {
	k9fxx08x0c_bad_blocks[block / 8] |= 1 << (block % 8);
}

static bool k9fxx08x0c_bad_block_get(uint16_t block) {
	return !!(k9fxx08x0c_bad_blocks[block / 8] & (1 << (block % 8)));
}

// A block is bad if the first byte of the spare area of the first page and/or of the second page is not 0xFF
static bool k9fxx08x0c_read_is_block_bad(k9fxx08x0c_t *k9fxx08x0c, uint16_t block) {
	uint8_t canary[2] = {0xAA, 0xAA};

	if (!k9fxx08x0c_read_page_part(k9fxx08x0c, block, 0, k9fxx08x0c->page_size, 1, &canary[0]))
		return TRUE;
	if (!k9fxx08x0c_read_page_part(k9fxx08x0c, block, 1, k9fxx08x0c->page_size, 1, &canary[1]))
		return TRUE;

	return (canary[0] != 0xFF) || (canary[1] != 0xFF);
}

static void k9fxx08x0c_write_block_is_bad(k9fxx08x0c_t *k9fxx08x0c, uint16_t block) {
	uint8_t marker = 0x00;

	k9fxx08x0c_write_page_part(k9fxx08x0c, block, 0, k9fxx08x0c->page_size, 1, &marker);
	k9fxx08x0c_write_page_part(k9fxx08x0c, block, 1, k9fxx08x0c->page_size, 1, &marker);

	k9fxx08x0c_bad_block_set(block);
}

static void k9fxx08x0c_bad_blocks_read(k9fxx08x0c_t *k9fxx08x0c) {
	memset(k9fxx08x0c_bad_blocks, 0, sizeof(k9fxx08x0c_bad_blocks));

	for (uint16_t i = 0; i < k9fxx08x0c->block_count; i++)
		if (k9fxx08x0c_read_is_block_bad(k9fxx08x0c, i))
			k9fxx08x0c_bad_block_set(i);
}


// -- Public NAND flash functions --

bool k9fxx08x0c_init(k9fxx08x0c_t *k9fxx08x0c) {
	memset(k9fxx08x0c, 0, sizeof(k9fxx08x0c_t));

	DDRC  = 0xF7; // CLE, ALE, /WE, /RE and /CE are output, R/B is input
	PORTC = 0x0F; // R/B with pull-up, high on /WE, /RE and /CE, low on CLE and ALE

	if (!k9fxx08x0c_is_ready())
		return FALSE;

	if (!k9fxx08x0c_read_id(k9fxx08x0c->id))
		return FALSE;

	// Decode ID information
	k9fxx08x0c_info_t info;
	k9fxx08x0c_get_information(k9fxx08x0c, &info);

	k9fxx08x0c->page_size       = ((uint16_t)info.size_page) * 1024;
	k9fxx08x0c->pages_per_block = info.size_block / info.size_page;
	k9fxx08x0c->block_count     = (((uint32_t)info.plane_size) * 1024) / (info.size_block * 8);

	k9fxx08x0c->page_spare_size = (k9fxx08x0c->page_size / 512) * info.size_spare;
	k9fxx08x0c->page_full_size  = k9fxx08x0c->page_size + k9fxx08x0c->page_spare_size;

	k9fxx08x0c->size      = ((uint32_t)k9fxx08x0c->page_size     ) * k9fxx08x0c->pages_per_block * k9fxx08x0c->block_count;
	k9fxx08x0c->full_size = ((uint32_t)k9fxx08x0c->page_full_size) * k9fxx08x0c->pages_per_block * k9fxx08x0c->block_count;

	// Sanity checks
	if ((info.maker_code != K9FXX08X0C_MAKER_CODE) || (info.device_code != K9FXX08X0C_DEVICE_CODE))
		return FALSE;
	if (k9fxx08x0c->block_count != K9FXX08X0C_BLOCK_COUNT)
		return FALSE;

	k9fxx08x0c_bad_blocks_read(k9fxx08x0c);

	return TRUE;
}

void k9fxx08x0c_get_information(const k9fxx08x0c_t *k9fxx08x0c, k9fxx08x0c_info_t *k9fxx08x0c_info) {
	k9fxx08x0c_info->maker_code  = k9fxx08x0c->id[0];
	k9fxx08x0c_info->device_code = k9fxx08x0c->id[1];

	k9fxx08x0c_info->chip_number = 1 <<  ( k9fxx08x0c->id[2]       & 0x03);
	k9fxx08x0c_info->cell_type   = 1 << (((k9fxx08x0c->id[2] >> 2) & 0x03) + 1);

	k9fxx08x0c_info->simul_pages = 1 << ((k9fxx08x0c->id[2] >> 4) & 0x03);

	k9fxx08x0c_info->interleave = !!(k9fxx08x0c->id[2] & 0x40);
	k9fxx08x0c_info->cache      = !!(k9fxx08x0c->id[2] & 0x80);

	k9fxx08x0c_info->size_page  = 1 <<  ( k9fxx08x0c->id[3]       & 0x03);
	k9fxx08x0c_info->size_block = 1 << (((k9fxx08x0c->id[3] >> 4) & 0x03) + 6);
	k9fxx08x0c_info->size_spare = 1 << (((k9fxx08x0c->id[3] >> 2) & 0x01) + 3);

	k9fxx08x0c_info->organization = 1 << (((k9fxx08x0c->id[3] >> 6) & 0x01) + 3);

	k9fxx08x0c_info->plane_count = 1 <<  ((k9fxx08x0c->id[4] >> 2) & 0x03);
	k9fxx08x0c_info->plane_size  = 1 << (((k9fxx08x0c->id[4] >> 4) & 0x07) + 6);

	uint8_t serial_delay = ((k9fxx08x0c->id[4] & 0x80) >> 6) | ((k9fxx08x0c->id[4] & 0x08) >> 3);
	k9fxx08x0c_info->serial_delay = (serial_delay == 0) ? 50 : ((serial_delay == 2) ? 25 : 0);
}

bool k9fxx08x0c_is_write_protected(k9fxx08x0c_t *k9fxx08x0c) {
	return !(k9fxx08x0c_read_status() & 0x80);
}

uint32_t k9fxx08x0c_address(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page, uint16_t offset) {
	return (((uint32_t)block * k9fxx08x0c->pages_per_block + (uint32_t)page) << 12) + (offset & 0x0FFF);
}

bool k9fxx08x0c_read_page(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page, uint8_t *data) {
	return k9fxx08x0c_read_page_part(k9fxx08x0c, block, page, 0, k9fxx08x0c->page_size, data);
}

bool k9fxx08x0c_read_page_part(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page,
                               uint16_t offset, uint16_t n, uint8_t *data) {

	// Sanity checks
	if ((offset + n) > k9fxx08x0c->page_full_size)
		return FALSE;
	if (k9fxx08x0c_is_block_bad(k9fxx08x0c, block))
		return FALSE;

	uint32_t address = k9fxx08x0c_address(k9fxx08x0c, block, page, offset);

	k9fxx08x0c_busy_wait();
	k9fxx08x0c_set_chip_enable();

	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_READ_ADDRESS);
	k9fxx08x0c_send_address_full(address);

	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_READ_START);
	k9fxx08x0c_busy_wait();
	k9fxx08x0c_receive_data(n, data);

	k9fxx08x0c_busy_wait();
	k9fxx08x0c_clear_chip_enable();

	return TRUE;
}

bool k9fxx08x0c_is_block_bad(k9fxx08x0c_t *k9fxx08x0c, uint16_t block) {
	return k9fxx08x0c_bad_block_get(block);
}

bool k9fxx08x0c_erase_block(k9fxx08x0c_t *k9fxx08x0c, uint16_t block) {
	// Sanity checks
	if (k9fxx08x0c_is_write_protected(k9fxx08x0c) || k9fxx08x0c_is_block_bad(k9fxx08x0c, block))
		return FALSE;

	uint32_t address = k9fxx08x0c_address(k9fxx08x0c, block, 0, 0);

	k9fxx08x0c_busy_wait();
	k9fxx08x0c_set_chip_enable();

	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_ERASE_ADDRESS);
	k9fxx08x0c_send_address_row(address);

	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_ERASE_START);

	k9fxx08x0c_busy_wait();
	k9fxx08x0c_clear_chip_enable();

	// If erasing failed, mark the block as bad
	bool success = k9fxx08x0c_status_success();
	if (!success)
		k9fxx08x0c_write_block_is_bad(k9fxx08x0c, block);

	return success;
}

static bool k9fxx08x0c_write_page_part(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page,
                                       uint16_t offset, uint16_t n, uint8_t *data) {

	// Sanity checks
	if ((offset + n) > k9fxx08x0c->page_full_size)
		return FALSE;
	if (k9fxx08x0c_is_write_protected(k9fxx08x0c) || k9fxx08x0c_is_block_bad(k9fxx08x0c, block))
		return FALSE;

	uint32_t address = k9fxx08x0c_address(k9fxx08x0c, block, page, offset);

	k9fxx08x0c_busy_wait();
	k9fxx08x0c_set_chip_enable();

	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_WRITE_ADDRESS);
	k9fxx08x0c_send_address_full(address);

	k9fxx08x0c_busy_wait();
	k9fxx08x0c_send_data(n, data);
	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_WRITE_START);

	k9fxx08x0c_busy_wait();
	k9fxx08x0c_clear_chip_enable();

	// If writing failed, mark the block as bad
	bool success = k9fxx08x0c_status_success();
	if (!success)
		k9fxx08x0c_write_block_is_bad(k9fxx08x0c, block);

	return success;
}

bool k9fxx08x0c_write_page(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page, uint8_t *data) {
	return k9fxx08x0c_write_page_part(k9fxx08x0c, block, page, 0, k9fxx08x0c->page_size, data);
}

bool k9fxx08x0c_verify_page_part(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page,
                                 uint16_t offset, uint16_t n, uint8_t *data, bool critical) {

	// Sanity checks
	if ((offset + n) > k9fxx08x0c->page_full_size)
		return FALSE;
	if (k9fxx08x0c_is_block_bad(k9fxx08x0c, block))
		return FALSE;

	uint32_t address = k9fxx08x0c_address(k9fxx08x0c, block, page, offset);

	k9fxx08x0c_busy_wait();
	k9fxx08x0c_set_chip_enable();

	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_READ_ADDRESS);
	k9fxx08x0c_send_address_full(address);

	k9fxx08x0c_send_command(K9FXX08X0C_COMMAND_READ_START);
	k9fxx08x0c_busy_wait();

	bool match = k9fxx08x0c_verify_data(n, data);

	k9fxx08x0c_busy_wait();
	k9fxx08x0c_clear_chip_enable();

	if (!match && critical)
		k9fxx08x0c_write_block_is_bad(k9fxx08x0c, block);

	return match;
}

bool k9fxx08x0c_verify_page(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page, uint8_t *data, bool critical) {
	return k9fxx08x0c_verify_page_part(k9fxx08x0c, block, page, 0, k9fxx08x0c->page_size, data, critical);
}
