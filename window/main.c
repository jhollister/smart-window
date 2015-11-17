#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include "nrf24.h"
#include "adc.h"
#include "scheduler.h"

#define F_CPU 8000000UL // 8 MHz
#include <util/delay.h>

enum window_status {
    NO_CONN,
    CLOSED,
    OPEN,
    CLOSING,
    OPENING
};

static char send_buffer[3];
static int8_t temp_out;
static int8_t temp_in;
static uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
static uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};

enum temp_states { TEMP_GET, TEMP_WAIT };
int tick_temp(int state) {
    static uint16_t adc_in;
    static float voltage;
    static float temperature;
    switch (state) {
        case TEMP_GET:
            adc_in = ADC - 40;
            PORTD = adc_in & 0xFF;
            PORTC = (adc_in >> 8) & 0x03;
            voltage = adc_in * 4.88;
            temperature = (voltage - 500.0) / 10.0;
            temperature = (temperature * 9.0 / 5.0) + 32.0;
            temp_in = (char)temperature;
            temp_out = 55;
            send_buffer[0] = temp_in;
            send_buffer[1] = temp_out;
            send_buffer[2] = OPEN;
            nrf24_send(send_buffer);
            state = TEMP_WAIT;
            break;
        case TEMP_WAIT:
            if (!nrf24_isSending()) {
               state = TEMP_GET;
            }
            break;
        default:
            state = TEMP_GET;
            break;
    }
    return state;
}

int main() {
    DDRD = 0xFF; PORTD = 0x00;
    DDRC = 0xFF; PORTC = 0x00;
    adc_init();
    nrf24_init();
    nrf24_config(2, 3); 
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);

    /* define tasks */
    tasksNum = 1; // declare number of tasks
    task tsks[1]; // initialize the task array
    tasks = tsks; // set the task array

    uint8_t i = 0;
    tasks[i].state = -1;
    tasks[i].period = 1000;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_temp;

    TimerSet(1000);
    TimerOn();
    
    while(1) {}
}
