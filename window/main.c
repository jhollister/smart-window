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
#include "adc.h"
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

enum auto_states { AUTO_OFF, AUTO_ON };
int tick_auto(int state) {
    switch (state) {
        case AUTO_OFF:
            if (_auto) {
                state = AUTO_ON;
            }
            break;
        case AUTO_ON:
            if (!_auto) {
                state = AUTO_OFF;
            }
            else {
                if (_status == OPEN) {
                    if ((_temp_in < _temp_min)) {
                        _input = INPUT_CLOSE;
                    }
                    else if ((_temp_out > _temp_max)) {
                        _input = INPUT_CLOSE;
                    }
                }
                else if (_status == CLOSED) {
                    if ( (_temp_out <= _temp_max) && (_temp_in >= _temp_max) ) {
                        _input = INPUT_OPEN;
                    }
                }
            }
            break;
        default:
            state = AUTO_OFF;
            break;
    }
    return state;
}

enum inputs_states { IN_WAIT, IN_CLOSE, IN_OPEN, IN_ALL, IN_PRESS };
int tick_input(int state) {
    static uint8_t ticks = 0;
    static const uint8_t PRESS_TIME = 30;
    switch (state) {
        case IN_WAIT:
            if ( CLOSE_IN() && _status != CLOSED) {
                _auto = 0;
                _input = INPUT_CLOSE;
                state = IN_CLOSE;
            }
            else if ( OPEN_IN() && _status != OPEN ) {
                _auto = 0;
                _input = INPUT_OPEN;
                state = IN_OPEN;
            }
            else if (_auto) {
                if (_status == OPEN) {
                    if (_temp_out > _temp_max) {
                        _input = INPUT_CLOSE;
                        state = IN_CLOSE;
                    }
                    else if (_temp_in < _temp_min) {
                        _input = INPUT_CLOSE;
                        state = IN_CLOSE;
                    }
                }
                else if (_status == CLOSED) {
                    if ( (_temp_out <= _temp_max) && (_temp_in >= _temp_max) ) {
                        _input = INPUT_OPEN;
                        state = IN_OPEN;
                    }
                }
            }

            break;
        case IN_CLOSE:
            if ( !CLOSE_IN() && ticks > PRESS_TIME) {
                _input = INPUT_STOP;
                state = IN_WAIT;
            }
            else if ( !CLOSE_IN() && ticks < PRESS_TIME) {
                _input = INPUT_CLOSE_ALL;
                ticks = 0;
                state = IN_ALL;
            }
            else {
                ticks = ticks <= PRESS_TIME ? ticks + 1 : ticks;
            }
            break;
        case IN_OPEN:
            if ( !OPEN_IN() && ticks > PRESS_TIME) {
                _input = INPUT_STOP;
                state = IN_WAIT;
            }
            else if ( !OPEN_IN() && ticks < PRESS_TIME) {
                _input = INPUT_OPEN_ALL;
                ticks = 0;
                state = IN_ALL;
            }
            else {
                ticks = ticks <= PRESS_TIME ? ticks + 1 : ticks;
            }
            break;
        case IN_ALL:
            // Stay in this state until a button is pressed or status is changed
            if ( OPEN_IN() || CLOSE_IN() || _status == OPEN || _status == CLOSED ) {
                state = IN_PRESS;
                _input = INPUT_STOP;
            }
            break;
        case IN_PRESS:
            // Wait for button to be unpressed
            if ( !OPEN_IN() && !CLOSE_IN() ) {
                state = IN_WAIT;
            }
            break;
        default:
            state = IN_WAIT;
            break;
    }
    return state;
}

/** 
 *  Motor state machine
 *  Handles the opening and closing of the window.
 *  Keeps track of the postion of the window 
 **/
enum motor_states { MOTOR_WAIT, MOTOR_STEP, MOTOR_OPEN, MOTOR_CLOSE };
int tick_motor(int state) {
    static uint8_t step_val = 0;
    static uint8_t steps = 0;
    static uint8_t revs = 0;
    switch (state) {
        case MOTOR_WAIT:
            PORTC = SetBit(PORTC, SLEEP_PIN, 0);
            if ( _input == INPUT_CLOSE ) {
                PORTC = SetBit(PORTC, SLEEP_PIN, 1);
                PORTC = SetBit(PORTC, DIR_PIN, CLOSE_DIR);
                _status = CLOSING;
                state = MOTOR_CLOSE;
            }
            else if ( _input == INPUT_OPEN ) {
                PORTC = SetBit(PORTC, SLEEP_PIN, 1);
                PORTC = SetBit(PORTC, DIR_PIN, OPEN_DIR);
                _status = OPENING;
                state = MOTOR_OPEN;
            }
            break;
        case MOTOR_OPEN:
            if ( revs >= REV_OPEN ) {
                _status = OPEN;
                state = MOTOR_WAIT;
            }
            else if ( _input == INPUT_STOP ) {
                _status = OPEN_PARTIAL;
                state = MOTOR_WAIT;
            }
            else {
                step_val = !step_val;
                PORTC = SetBit(PORTC, STEP_PIN, step_val);
                if (step_val == 0) {
                    steps++;
                }
                if (steps == 200) {
                    steps = 0;
                    revs++;
                }
            }
            break;
        case MOTOR_CLOSE:
            if ( revs == 0 ) {
                _status = CLOSED;
                state = MOTOR_WAIT;
            }
            else if ( _input == INPUT_STOP ) {
                _status = OPEN;
                state = MOTOR_WAIT;
            }
            else {
                step_val = !step_val;
                PORTC = SetBit(PORTC, STEP_PIN, step_val);
                if (step_val == 0) {
                    steps--;
                }
                if (steps == 0) {
                    steps = STEPS_REV;
                    revs--;
                }
            }
            break;
        default:
            state = MOTOR_WAIT;
            break;
    }
    return state;
}

enum nrf_states { NRF_RCV, NRF_SEND, NRF_WAIT };
int tick_send(int state) {
    static uint8_t first_time = 1;
    static uint8_t temp;
    switch (state) {
        case NRF_SEND:
            _send_buffer[0] = _temp_in;
            _send_buffer[1] = _temp_out;
            _send_buffer[2] = _status == OPEN_PARTIAL ? OPEN : _status;
            _send_buffer[3] = _auto;
            if ((_status != CLOSING && _status != OPENING)) {
                first_time = 1;
                state = NRF_WAIT;
                nrf24_send(_send_buffer);
            }
            else if ((_status == CLOSING || _status == OPENING) && first_time) {
                nrf24_send(_send_buffer);
                state = NRF_WAIT;
                first_time = 0;
            }
            break;
        case NRF_WAIT:
            if (!nrf24_isSending()) {
                state = NRF_SEND;
                nrf24_powerUpRx();
            }
            break;
        default:
            state = NRF_SEND;
            break;
    }
    return state;
}

int tick_rcv(int state) {
    switch(state) {
        case NRF_RCV:
            if (nrf24_dataReady() && !nrf24_isSending()) {
                nrf24_getData(_rcv_buffer);
                _temp_max = _rcv_buffer[0];
                _temp_min = _rcv_buffer[1];
                _rf_input = _rcv_buffer[2];
                if (_rcv_buffer[3]) {
                    _auto = 1;
                }
            }
            break;
        default:
            state = NRF_RCV;
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
    nrf24_init();
    nrf24_config(6, 4);
    nrf24_tx_address(_tx_address);
    nrf24_rx_address(_rx_address);

    _temp_in = therm_read_temperature(1);
    _temp_out = therm_read_temperature(0);

    /* define tasks */
    tasksNum = 5; // declare number of tasks
    task tsks[5]; // initialize the task array
    tasks = tsks; // set the task array

    uint8_t i = 0;
    tasks[i].state = TEMP_GET;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_temp;
    i++;
    tasks[i].state = NRF_SEND;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_send;
    i++;
    tasks[i].state = IN_WAIT;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_input;
    i++;
    tasks[i].state = NRF_RCV;
    tasks[i].period = 10;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_rcv;
    i++;
    tasks[i].state = MOTOR_WAIT;
    tasks[i].period = 1;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_motor;

    TimerSet(1);
    TimerOn();

    while(1) {}
}
