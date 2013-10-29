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

#ifndef USART_H_
#define USART_H_

#include <stdio.h>

/** Initialize the USART0.
 *
 *  Note: This also binds stdout and stdin to the USART0, so that stdio functions can be used.
 *        For example, printf() will print to the USART0.
 */
void usart0_init(void);

/** Send a byte through the USART0. */
void usart0_put(unsigned char c);

/** Receive a byte through the USART0.
 *
 *  @return The byte read as an unsigned char cast to an int, _FDEV_ERR on error,
            or _FDEV_EOF is no byte is available to be read.
 */
int usart0_get(void);

/** Receive a byte through the USART0, blockingly.
 *
 *  Blocks until a byte has been read or an error occured.
 *
 *  @return The byte read as an unsigned char cast to an int or _FDEV_ERR on error.
 */
int usart0_get_wait(void);

/** Enable echo mode: All read characters will be send back through the USART0. */
void usart0_enable_echo(void);

/** Disable echo mode (default). */
void usart0_disable_echo(void);

#endif /* USART_H_*/
