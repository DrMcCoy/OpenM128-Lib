/* util - Utility macros
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

#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>
#include <string.h>

#ifndef MAX
	#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
	#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef CLIP
	#define CLIP(x, lower, upper) MAX((lower), MIN((x), (upper)))
#endif

#ifndef ABS
	#define ABS(x) (((x) < 0) ? (-(x)) : (x))
#endif

#ifndef SWAP
	#define SWAP(x,y) do { \
		unsigned char swap_temp[sizeof(x) == sizeof(y) ? (signed)sizeof(x) : -1]; \
		memcpy(swap_temp,&y,sizeof(x)); \
		memcpy(&y,&x,       sizeof(x)); \
		memcpy(&x,swap_temp,sizeof(x)); \
	} while(0)
#endif

#ifndef ARRAYSIZE
	#define ARRAYSIZE(x) ((int)(sizeof(x) / sizeof(x[0])))
#endif

uint32_t sqrt_integer_rounded(uint32_t a_nInput);

/** Pad a string with character c to make it at least length characters long.
 *
 *  Make sure str can hold at least length + 1 characters!
 */
void pad_string(char *str, unsigned int length, char c);

#endif /* UTIL_H_ */
