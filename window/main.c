/**
 * Author: James Hollister
 * Partner: Roberto Pasillas
 *
 * Main state machine functionality for window
 */
#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include "nrf24.h"
#include "scheduler.h"
#include "ds18b20.h"
#include "bit.h"

#define F_CPU 8000000UL // 8 MHz
#include <util/delay.h>

#define MAX_OUT_TEMP 80
#define STEP_PIN     0
#define DIR_PIN      1
#define SLEEP_PIN    2

#define CLOSE_PIN    1
#define OPEN_PIN     0

#define CLOSE_DIR    1
#define OPEN_DIR     0

// Steps in a revolution
#define STEPS_REV    200
// Revolutions to open or close fully
#define REV_OPEN     30

#define NO_CONN 0
#define CLOSED  1
#define OPEN    2
#define CLOSING 3
#define OPENING 4
#define OPEN_PARTIAL 5

enum inputs {
    INPUT_CLOSE_ALL,
    INPUT_CLOSE,
    INPUT_OPEN_ALL,
    INPUT_OPEN,
    INPUT_STOP
};

/* State machine variables */
static uint8_t _send_buffer[4];
static uint8_t _rcv_buffer[4];
static int8_t _temp_out;
static int8_t _temp_in;
static int8_t _temp_max = 75;
static int8_t _temp_min = 63;
static uint8_t _status = CLOSED;
static uint8_t _tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
static uint8_t _rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
static uint8_t _input = INPUT_STOP;
static uint8_t _rf_input = NO_CONN;
static uint8_t _auto = 0;

#define CLOSE_IN() ( _rf_input == CLOSING || (PIND & 0x03) == 1 )
#define OPEN_IN() ( _rf_input == OPENING || (PIND & 0x03) == 2 )
/* State machines */


enum nrf_states { NRF_RCV, NRF_SEND, NRF_WAIT };
int tick_send(int state) {
    static uint8_t val = 1;
    switch (state) {
        case NRF_SEND:
            /*if (nrf24_dataReady() && !nrf24_isSending()) {*/
                /*nrf24_getData(_rcv_buffer);*/
                /*if (_rcv_buffer[0] == 0x25) {*/
                    /*PORTB = SetBit(PORTB, 2, val);*/
                    /*val = !val;*/
                /*}*/
            /*}*/
            _send_buffer[0] = _temp_in;
            _send_buffer[1] = _temp_out;
            _send_buffer[2] = _status == OPEN_PARTIAL ? OPEN : _status;
            _send_buffer[3] = _auto;
            nrf24_send(_send_buffer);
            while(nrf24_isSending());
            /*nrf24_powerUpRx();*/
            break;
        default:
            state = NRF_SEND;
            break;
    }
    return state;
}

enum temp_states { TEMP_GET };
int tick_temp(int state) {
    switch (state) {
        case TEMP_GET:
            // Don't check temperature while window is opening or closing
            if (_status == OPEN || _status == CLOSED || _status == OPEN_PARTIAL) {
                _temp_in = therm_read_temperature(1);
                _temp_out = therm_read_temperature(0);
            }
            break;
        default:
            state = TEMP_GET;
            break;
    }
    return state;
}

int main() {
    DDRD = 0x00; PORTD = 0xFF;
    DDRC = 0xFF; PORTC = 0x00;
    DDRB = 0xFF; PORTB = 0x00;
    nrf24_init();
    nrf24_config(6, 4);
    nrf24_tx_address(_tx_address);
    nrf24_rx_address(_rx_address);

    _temp_in = therm_read_temperature(1);
    _temp_out = therm_read_temperature(0);

    /* define tasks */
    tasksNum = 2; // declare number of tasks
    task tsks[2]; // initialize the task array
    tasks = tsks; // set the task array

    uint8_t i = 0;
    tasks[i].state = TEMP_GET;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_temp;
    i++;
    tasks[i].state = NRF_SEND;
    tasks[i].period = 500;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_send;

    TimerSet(100);
    TimerOn();

    while(1) {}
}
