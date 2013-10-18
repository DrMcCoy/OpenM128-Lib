/* keys - Reading the keys on the Waveshare OpenM128 board
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

#ifndef KEYS_H_
#define KEYS_H_

#include "openm128/types.h"

/** NOTE: Even though the keys on the OpenM128 boards are labeled "KEY1", "KEY2, "KEY3" and "KEY4",
 *        we start counting at 0.
 *
 *        Therefore:
 *        KEY1 = key 0, bit 0
 *        KEY2 = key 1, bit 1
 *        KEY3 = key 2, bit 2
 *        KEY4 = key 3, bit 3
 */

/** Utility macro to convert 4 separate booleans (for each key) into a key bitmask. */
#define KEYS_MASK(KEY0, KEY1, KEY2, KEY3) (((KEY0) ? 0x01 : 0x00) |\
                                           ((KEY1) ? 0x02 : 0x00) |\
                                           ((KEY2) ? 0x04 : 0x00) |\
                                           ((KEY3) ? 0x08 : 0x00))

/** Initialize the keys on the OpenM128 boards.
 *
 *  @param mask A mask to specify which keys should be initialized.
 *              If bit n (0-3) is set, key n will be initialized.
 *              If bit n (0-3) is not set, key n will not be initialized.
 */
void keys_init(uint8_t mask);

/** Return the state of a specific key.
 *
 *  @param key  The key (0-3) to query.
 *
 *  @return TRUE if the key is pressed down, FALSE if the key is unpressed.
 */
bool keys_get(uint8_t key);

/** Return the state of several keys according to a mask.
 *
 *  @param mask A mask to specific which keys to query.
 *              If bit n (0-3) is set, key n will be queried.
 *              If bit n (0-3) is not set, key n will be ignored.
 *
 *  @return A bitmask of the states of each keys.
 *          If key n (0-3) is pressed down (and bit n of mask is set), bit n is 1.
 *          If key n (0-3) is unpressed (and bit n of mask is set), bit n is 0.
 */
uint8_t keys_get_mask(uint8_t mask);

#endif /* KEYS_H_ */
