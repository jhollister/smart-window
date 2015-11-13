#include <avr/io.h>
#include <stdint.h>
#include "nrf24.h"
#include "lcd.h"

uint8_t temp;
uint8_t q = 0;
uint8_t data_array[4];
uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};

int main() {
    DDRD = 0xFF; PORTD = 0;
    DDRC = 0xFF; PORTC = 0;
    LCD_init();
    LCD_DisplayString(17, "Receiving...");
    // init hardware pins
    nrf24_init();

    /* Channel #2, payload length: 4 */
    nrf24_config(2, 6);

    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);

    while(1) {
        if (nrf24_dataReady()) {
            nrf24_getData(data_array);
            LCD_DisplayString(1, data_array);
            LCD_Cursor(0);
        }
    }
}
