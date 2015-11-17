/* ADC interface file */
#ifndef ADC_H
#define ADC_H

#include <avr/io.h>


void adc_init(void) {
    // ADEN: Enables ADC
    // ADSC: Starts ADC
    // ADATE: Enables auto-triggering. Allowing for constant conversion
    ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADATE);
}

// Pins on PORTA are used as input for A2D conversion
// The default channel is 0 (PA0)
// The value of pinNum determines the pin on PORTA
// used for A2D conversion
// Valid values range between 0 and 7, where the value
// represents the desired pin for A2D conversion
void adc_set_pin(unsigned char pinNum) {
	ADMUX = (pinNum <= 0x07) ? pinNum : ADMUX;
	// Allow channel to stabilize
	static unsigned char i = 0;
	for ( i=0; i<20; i++ ) {
		asm("nop");
	}
}

#endif
