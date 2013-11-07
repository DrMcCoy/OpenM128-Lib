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

/** @file k9fxx08x0c.h
 *  This code is *not* ready for serious production use. At the very least,
 *  some form of error detection / correct scheme needs to be implemented.
 */

#ifndef K9FXX08X0C_H_
#define K9FXX08X0C_H_

#include "openm128/types.h"

typedef struct {
	uint8_t id[5]; ///< The ID. Used in k9fxx08x0c_get_information().

	uint16_t page_size;       ///< Size of a page without the spare area.
	uint16_t pages_per_block; ///< Number of pages per block.
	uint16_t block_count;     ///< Number of blocks on the device.

	uint16_t page_spare_size; ///< Size of the spare area in a page.
	uint16_t page_full_size;  ///< Size of a page including the spare area.

	uint32_t size;      ///< Size of the memory in bytes, without the spare area.
	uint32_t full_size; ///< Size of the memory in bytes, including the spare area.
} k9fxx08x0c_t;

/** Mostly useless information about the flash memory. */
typedef struct {
	uint8_t maker_code;
	uint8_t device_code;

	uint8_t chip_number;
	uint8_t cell_type;

	uint8_t simul_pages; ///< Number of pages that can be programmed simultaniously.

	bool interleave; ///< Support for interleaving programming between multiple chips.
	bool cache;      ///< Support for cached programming.

	uint8_t  size_page;  ///< Page size in KB, without spare area.
	uint16_t size_block; ///< Block size in KB, without spare area.
	uint8_t  size_spare; ///< Number of bytes in the spare area per 512 real data.

	uint8_t organization;

	uint8_t serial_delay; ///< Speed of serial access in ns.

	uint8_t  plane_count; ///< Number of planes.
	uint16_t plane_size; ///< Size of a plane in Mb.
} k9fxx08x0c_info_t;


/** Initialize the NAND flash memory. */
bool k9fxx08x0c_init(k9fxx08x0c_t *k9fxx08x0c);

/** Return some mostly useless information about the flash device. */
void k9fxx08x0c_get_information(const k9fxx08x0c_t *k9fxx08x0c, k9fxx08x0c_info_t *k9fxx08x0c_info);

/** Is the NAND flash write-protected? */
bool k9fxx08x0c_is_write_protected(k9fxx08x0c_t *k9fxx08x0c);

/** Read a whole page, excluding the spare area.
 *
 *  Note: The reduce stress on the cells, all pages within a block should be read
 *        equally as often and frequent.
 *
 *  @param k9fxx08x0c The device to read from.
 *  @param block      The block to read from.
 *  @param page       The page within the block to read from.
 *  @param data       Store the data here.
 *
 *  @return TRUE if reading the data was successful.
 */
bool k9fxx08x0c_read_page(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page, uint8_t *data);

/** Read part of a page.
 *
 *  Note: The reduce stress on the cells, all pages within a block and all bytes within a page
 *        should be read equally as often and frequent.
 *
 *  @param k9fxx08x0c The device to read from.
 *  @param block      The block to read from.
 *  @param page       The page within the block to read from.
 *  @param offset     The offset within the page from where to start reading.
 *  @param n          The number of bytes to read.
 *  @param data       Store the data here.
 *
 *  @return TRUE if reading the data was successful.
 */
bool k9fxx08x0c_read_page_part(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page,
                               uint16_t offset, uint16_t n, uint8_t *data);

/** Check whether a block has been marked as bad.
 *
 *  The writing, erasing and reading functions will check if a block has been marked as bad
 *  and abort with a fail status in that case. Likewise, if writing to or erasing a good block
 *  fails, it will automatically be marked as bad. Once a block has been marked as bad, it will
 *  stay that was; it can't ever be recovered again.
 */
bool k9fxx08x0c_is_block_bad(k9fxx08x0c_t *k9fxx08x0c, uint16_t block);

/** Erase a whole block.
 *
 *  If erasing fails, the block will be marked as bad.
 *
 *  @param k9fxx08x0c The device to operate on.
 *  @param block      The block to erase.
 *
 *  @return TRUE if erasing was successful.
 */
bool k9fxx08x0c_erase_block(k9fxx08x0c_t *k9fxx08x0c, uint16_t block);

/** Write a whole page, excluding the spare area.
 *
 *  If writing fails, the block will be marked as bad.
 *
 *  Note: To avoid destructive write disturbance (other bits in the block being set), pages in a block *really*
 *        should be written sequential. I.e. don't write page 0 after writing to page 1 without erasing the
 *        whole block in between. Otherwise, data retention can't be guaranteed!
 *
 *  @param k9fxx08x0c The device to write to.
 *  @param block      The block to write to.
 *  @param page       The page within the block to write to.
 *  @param data       The data to write.
 *
 *  @return TRUE if writing the data was successful.
 */
bool k9fxx08x0c_write_page(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page, uint8_t *data);

/** Read the contents of a whole page and compare that it matches with data.
 *
 *  @param k9fxx08x0c The device to operate on.
 *  @param block      The block to operate on.
 *  @param page       The page within the block to compare.
 *  @param data       The data to to compare the page with.
 *  @param critical   If TRUE and the device is not write-protected, mark the block as bad when comparison fails.
 *
 *  @return TRUE if the contents matches.
 */
bool k9fxx08x0c_verify_page(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page, uint8_t *data, bool critical);

/** Read the contents of part of a page and compare that it matches with data.
 *
 *  @param k9fxx08x0c The device to operate on.
 *  @param block      The block to operate on.
 *  @param page       The page within the block to compare.
 *  @param data       The data to to compare the page with.
 *  @param offset     The offset within the page from where to start comparing.
 *  @param n          The number of bytes to compare.
 *  @param critical   If TRUE and the device is not write-protected, mark the block as bad when comparison fails.
 *
 *  @return TRUE if the contents matches.
 */
bool k9fxx08x0c_verify_page_part(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page,
                                 uint16_t offset, uint16_t n, uint8_t *data, bool critical);

/** Return an unique address for this memory location on the flash device. */
uint32_t k9fxx08x0c_address(k9fxx08x0c_t *k9fxx08x0c, uint16_t block, uint16_t page, uint16_t offset);

#endif /* K9FXX08X0C_H_ */
