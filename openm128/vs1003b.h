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

#ifndef VS1003B_H_
#define VS1003B_H_

#include "openm128/types.h"

typedef enum {
	kVS1003BFormatNone = 0,
	kVS1003BFormatWAV,
	kVS1003BFormatMP3,
	kVS1003BFormatWMA,
	kVS1003BFormatMID,
	kVS1003BFormatUnknown
} vs1003b_format_t;

/** Initialize the VS1003B MP3/WMA/WAV/MID decoder. */
bool vs1003b_init(void);

/** Reset the VS1003B. */
void vs1003b_reset(void);

/** Is the VS1003B ready to receive more data? */
bool vs1003b_ready(void);

/** Feed the VS1003B data.
 *
 *  @param data  The data to write.
 *  @param count The number of bytes to write.
 *
 *  @return The number of bytes successfully written to the VS1003B.
 */
uint16_t vs1003b_feed_data(uint8_t *data, uint16_t count);

/** Stop playing/decoding. */
void vs1003b_stop(void);

/** Set the volume for the left and right output channel. */
void vs1003b_set_volume(uint8_t left, uint8_t right);

/** Return the current volume. */
void vs1003b_get_volume(uint8_t *left, uint8_t *right);

/** Return the number of seconds the current data has been decoded. */
uint16_t vs1003b_get_decode_time(void);

/** Return the currently playing format. */
vs1003b_format_t vs1003b_get_format(void);

/** Control the bass and treble control.
 *
 *  @param bass_amplitude   Bass enhancement in dB (0-15). 0 = off
 *  @param bass_freqlimit   Lower limit frequency of the bass enhancement in Hz (20-150). Granularity is 10Hz.
 *  @param treble_amplitude Treble control in 1.5dB steps (-8-7). 0 = off.
 *  @param treble_freqlimit Lower limit frequency of the treble control in kHz (0-15).
 *
 *  @return TRUE if the parameters were within valid range and the control was applied.
 */
bool vs1003b_set_bass_treble(uint8_t bass_amplitude, uint8_t bass_freqlimit, int8_t treble_amplitude, uint8_t treble_freqlimit);

#endif /* VS1003B_H_ */
