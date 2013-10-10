/* adc - Reading values from the analog-to-digital converter on the ATmega128
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

/** @file adc.h
 *  These functions read a value from the analog-to-digital converter on the ATmega128.
 *
 *  Please note the following:
 *    - The ADC provides a resolution of 10bit.
 *    - AVCC and GND need to be connected.
 *    - AVCC MUST NOT differ more than 0.3V from the ATmega128's VCC.
 *    - GND provides the minimum to be expected from the analog signal, the reference the maximum.
 *      If the input signal is near GND, the ADC values will be 0.
 *      If the input signal is near the reference, the ADC values will be 1023.
 *    - The AVR input pin voltage range MUST be respected, otherwise you'll fry the ATmega128:
 *      The signal MUST be below VCC + 0.5V and above -1V.
 *    - To decrease noise while using AVCC or the internal 2.56V as a reference, try connecting
 *      a small (100nF) capacitor between AREF and GND.
 *    - Channels 4-7 (pins 4-7 on port F) can't be used for keypad input while JTAG is enabled.
 */

#ifndef ADC_H_
#define ADC_H_

/** The source of the reference voltage used by the ADC. */
typedef enum {
	kADCReferenceAREF = 0x00, ///< The ADC uses the AREF pin as its reference.
	kADCReferenceAVCC = 0x01, ///< The ADC uses AVCC as its reference.
	kADCReference256V = 0x03  ///< The ADC uses its internal 2.56V reference.
} adc_reference_t;

/** Read a value from the ADC channel.
 *
 *  Conversion will take 38 ADC clock cycles. The ADC clock will be between 50kHz and 200kHz
 *  (depending on the AVR clock), so conversion will take between 19µs and 76µs, not taking
 *  calling overhead into account.
 *
 *  @param channel   The channel (0-7) to sample. Channel x corresponds to pin x on port F.
 *  @param reference The source of the reference voltage.
 *
 *  @return The converted value, 10bit resolution.
 */
uint16_t adc_get(uint8_t channel, adc_reference_t reference);

/** Read an averaged value from the ADC channel.
 *
 *  To remove noise, this function samples the ADC several times and average the
 *  result while removing outliers. Note that this is at least 10 times slower than
 *  unaveraged reading.
 */
uint16_t adc_get_average(uint8_t channel, adc_reference_t reference);

#endif /* ADC_H_ */