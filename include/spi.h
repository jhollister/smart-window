/* SPI Interface header file */
#include <avr/io.h>
#include <avr/interrupt.h>

volatile unsigned char spi_data = 0;

// Master code
void SPI_MasterInit(void) {
	// Set DDRB to have MOSI, SCK, and SS as output and MISO as input
	DDRB = (1 << 5) | (1 << 4) | (1 << 7);
	// Set SPCR register to enable SPI, enable master, and use SCK frequency
	// of fosc/16  (pg. 168)
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
	// Make sure global interrupts are enabled on SREG register (pg. 9)
//    sei();
}

void SPI_MasterTransmit(unsigned char cData, unsigned char ss) {
    // data in SPDR will be transmitted, e.g. SPDR = cData;
	// set SS low
    PORTB &= ~(1 << ss);
    SPDR = cData;
	while(!(SPSR & (1<<SPIF))) { // wait for transmission to complete
		;
	}
    // set SS high
    PORTB |= (1 << ss);
}

// Servant code
void SPI_ServantInit(void) {
	// set DDRB to have MISO line as output and MOSI, SCK, and SS as input
    DDRB = (1 << 6);
	// set SPCR register to enable SPI and enable SPI interrupt (pg. 168)
    SPCR = (1 << SPE) | (1 << SPIE);
	// make sure global interrupts are enabled on SREG register (pg. 9)
    sei();
}

ISR(SPI_STC_vect) {
    // this is enabled in with the SPCR register’s “SPI
    // Interrupt Enable”
	// SPDR contains the received data, e.g. unsigned char receivedData =
    // SPDR;
    static unsigned char val = (1 << 7);
    val = ~val;
    PORTD = ~val;

    spi_data = SPDR;
}
