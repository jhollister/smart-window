#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include "nrf24.h"

#define F_CPU 8000000UL // 8 MHz
#include <util/delay.h>

uint8_t temp;
uint8_t q = 0;
char data_array[6];
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};


int main() {
    nrf24_init();
    nrf24_config(2, 6); 
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);

    uint8_t hello = 1;
    
    while(1) {

        if (hello) {
            strcpy(data_array, "hello");
        }
        else {
            strcpy(data_array, "bye  ");
        }

        nrf24_send(data_array);
        while(nrf24_isSending());
        hello = !hello;

        _delay_ms(1000);
    }
}
