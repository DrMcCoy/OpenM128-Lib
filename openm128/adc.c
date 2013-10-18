/* adc - Reading values from the ADC on the ATmega128
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

#include "adc.h"
#include "util.h"

#ifndef F_CPU
	#error F_CPU needs to be defined for the AD Keypad to work
#endif

// The ADC clock needs to be between 50kHz and 200kHz
#if   F_CPU ==  1000000UL
	#define ADKEYPAD_PRESCALER 3       // Prescaler of   8 = ADC clock of 125.0kHz
#elif F_CPU ==  2000000UL
	#define ADKEYPAD_PRESCALER 4       // Prescaler of  16 = ADC clock of 125.0kHz
#elif F_CPU ==  4000000UL
	#define ADKEYPAD_PRESCALER 5       // Prescaler of  32 = ADC clock of 125.0kHz
#elif F_CPU ==  6000000UL
	#define ADKEYPAD_PRESCALER 5       // Prescaler of  32 = ADC clock of 187.5kHz
#elif F_CPU ==  7372800UL
	#define ADKEYPAD_PRESCALER 6       // Prescaler of  64 = ADC clock of 115.2kHz
#elif F_CPU ==  8000000UL
	#define ADKEYPAD_PRESCALER 6       // Prescaler of  64 = ADC clock of 125.0kHz
#elif F_CPU == 16000000UL
	#define ADKEYPAD_PRESCALER 7       // Prescaler of 128 = ADC clock of 125.0kHz
#else
	#error Unsupported system clock
#endif

static uint16_t adc_read() {
	// Start conversion
	ADCSRA |= (1 << ADSC);

	// Wait until ADIF is set (= conversion finished)
	while (!(ADCSRA & (1 << ADIF)));

	// clear ADIF (by writing 1 into it)
	ADCSRA |= (1 << ADIF);

	uint8_t adc_low  = ADCL;
	uint8_t adc_high = ADCH;

	return (adc_high << 8) | adc_low;
}

uint16_t adc_get(uint8_t channel, adc_reference_t reference) {
	// Select reference, right adjusted value, singled ended input on pin
	ADMUX = ((reference & 0x03) << REFS0) | (0 << ADLAR) | ((channel & 0x03) << MUX0);

	// Enable ADC, stop conversion, single shot mode, reset interrupt flag, disable interrupt, select prescaler
	ADCSRA = (1 << ADEN) | (0 << ADSC) | (0 << ADFR) | (1 << ADIF) | (0 << ADIE) | (ADKEYPAD_PRESCALER << ADPS0);

	// The first ADC result after changing input, reference and/or prescaler can be imprecise.
	// So we'll throw away that first result and read a new result.
	adc_read();

	uint16_t adc_value = adc_read();

	// Disable the ADC again
	ADCSRA &= ~(1 << ADEN);

	return adc_value;
}

#define ADC_AVERAGE_READ    10
#define ADC_AVERAGE_IGNORE   4
#define ADC_AVERAGE_USE    (ADC_AVERAGE_READ - (2*ADC_AVERAGE_IGNORE))
uint16_t adc_get_average(uint8_t pin, adc_reference_t reference) {
	uint16_t buf[ADC_AVERAGE_READ];

	// Read the ADC ADC_AVERAGE_READ times
	for (uint8_t i = 0; i < ADC_AVERAGE_READ; i++)
		buf[i] = adc_get(pin, reference);

	// Sort the read ADC values
	for (uint8_t i = 0; i < ADC_AVERAGE_READ-1; i++)
		for (uint8_t j = i + 1; j < ADC_AVERAGE_READ; j++)
			if (buf[i] > buf[j])
				SWAP(buf[i], buf[j]);

	// Average over the middle values
	uint16_t avg = 0;
	for (uint8_t i = 0; i < ADC_AVERAGE_USE; i++)
		avg += buf[ADC_AVERAGE_IGNORE + i];
	avg /= ADC_AVERAGE_USE;

	return avg;
}
