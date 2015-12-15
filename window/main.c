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
#include "adc.H"

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
static int8_t _temp_max;
static int8_t _temp_min;
static uint8_t _status = CLOSED;
static uint8_t _tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
static uint8_t _rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
static uint8_t _auto = 0;
static uint8_t _revs = REV_OPEN;
static uint8_t _steps = 0;
static uint8_t _no_force_sensor = 0;

#define CLOSE_IN() ( _rf_input == CLOSING || (PIND & 0x03) == 1 )
#define OPEN_IN() ( _rf_input == OPENING || (PIND & 0x03) == 2 )

int send_rx(uint8_t *buffer) {
    uint8_t result;
    nrf24_send(buffer);
    while (!nrf24_isSending());

    result = nrf24_lastMessageStatus();
    nrf24_powerUpRx();
    return result;
}

void window_close() {
    uint16_t wait = 1000;
    uint16_t force = ADC;
    uint16_t close_count = 0;
    if (force >= 100) {
        _status = CLOSED;
        _revs = 0;
        _steps = 0;
        return;
    }
    _send_buffer[2] = CLOSING;
    send_rx(_send_buffer);
    PORTC = SetBit(PORTC, DIR_PIN, CLOSE_DIR);
    PORTC = SetBit(PORTC, SLEEP_PIN, 1);
    while (force < 100 && !_no_force_sensor) {
        if (nrf24_dataReady()) {
            nrf24_getData(_rcv_buffer);
            PORTC = SetBit(PORTC, SLEEP_PIN, 0);
            _status = OPEN;
            return;
        }
        else if (!GetBit(PIND, 0) || !GetBit(PIND, 1)) {
            PORTC = SetBit(PORTC, SLEEP_PIN, 0);
            _status = OPEN;
            return;
        }
        PORTC = SetBit(PORTC, STEP_PIN, 1);
        therm_delay(us(wait));
        PORTC = SetBit(PORTC, STEP_PIN, 0);
        therm_delay(us(wait));
        _steps--;
        if (_steps == 0) {
            _revs = _revs > 0 ? _revs - 1 : _revs;
            _steps = STEPS_REV;
        }
        if (_revs == 0) {
            close_count++;
            if (close_count > 400) {
                _no_force_sensor = 1;
                break;
            }
        }
        wait = wait > 400 ? wait - 1 : wait;
        force = ADC;
    }
    _status = CLOSED;
    _revs = 0;
    _steps = 0;
    PORTC = SetBit(PORTC, SLEEP_PIN, 0);
}

void window_open() {
    uint16_t wait = 1000;
    if (_revs == REV_OPEN)
        return;
    _send_buffer[2] = OPENING;
    send_rx(_send_buffer);
    PORTC = SetBit(PORTC, DIR_PIN, OPEN_DIR);
    PORTC = SetBit(PORTC, SLEEP_PIN, 1);
    while (_revs < REV_OPEN && !_no_force_sensor) {
        if (nrf24_dataReady()) {
            nrf24_getData(_rcv_buffer);
            PORTC = SetBit(PORTC, SLEEP_PIN, 0);
            _status = OPEN;
            return;
        }
        else if (!GetBit(PIND, 0) || !GetBit(PIND, 1)) {
            PORTC = SetBit(PORTC, SLEEP_PIN, 0);
            _status = OPEN;
            return;
        }
        PORTC = SetBit(PORTC, STEP_PIN, 1);
        therm_delay(us(wait));
        PORTC = SetBit(PORTC, STEP_PIN, 0);
        therm_delay(us(wait));
        _steps++;
        if (_steps == STEPS_REV) {
            _revs++;
            _steps = 0;
        }
        wait = wait > 300 ? wait - 1 : wait;
    }
    _status = OPEN;
    PORTC = SetBit(PORTC, SLEEP_PIN, 0);
}
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
            else if (_status == CLOSED) {
                if (_temp_in > _temp_max && _temp_out < _temp_max) {
                    window_open();
                }
            }
            else if (_status == OPEN) {
                if (_temp_in < _temp_min) {
                    window_close();
                }
                else if (_temp_in > _temp_max && _temp_out > _temp_max) {
                    window_close();
                }
            }
            break;
        default:
            state = AUTO_OFF;
    }
    return state;
}

enum sensor_states { WAIT };
int tick_alert(int state) {
    static uint8_t val = 0;
    switch (state) {
        case WAIT:
            if (!GetBit(PIND, 0)) {
                _no_force_sensor = 0;
            }
            if (_no_force_sensor) {
                PORTB = SetBit(PORTB, 2, val);
                val = !val;
            }
            else {
                PORTB = SetBit(PORTB, 2, 0);
            }
            break;
        default:
            state = WAIT;
            break;
    }
    return state;
}


enum nrf_states { NRF_RCV, NRF_SEND, NRF_WAIT };
int tick_nrf(int state) {
    switch (state) {
        case NRF_SEND:
            if (nrf24_dataReady()) {
                nrf24_getData(_rcv_buffer);
                if (_rcv_buffer[0] == OPEN) {
                    if (_auto) {
                        _auto = 0;
                    }
                    else {
                        window_open();
                    }
                }
                else if (_rcv_buffer[0] == CLOSED) {
                    if (_auto) {
                        _auto = 0;
                    }
                    else {
                        window_close();
                    }
                }
                else if (_rcv_buffer[0] == 3) {
                    _auto = 1;
                    _temp_max = _rcv_buffer[1];
                    _temp_min = _rcv_buffer[2];
                }
            }
            _send_buffer[0] = _temp_in;
            _send_buffer[1] = _temp_out;
            if (_no_force_sensor) {
                _send_buffer[2] = -1;
            }
            else {
                _send_buffer[2] = _status == OPEN_PARTIAL ? OPEN : _status;
            }
            _send_buffer[3] = _auto;
            send_rx(_send_buffer);
            break;
        default:
            state = NRF_SEND;
            break;
    }
    return state;
}

enum temp_states { TEMP_INIT, TEMP_GET };
int tick_temp(int state) {
    switch (state) {
        case TEMP_GET:
            _temp_in = therm_read_temperature(1);
            _temp_out = therm_read_temperature(0);
            break;
        default:
            state = TEMP_INIT;
            break;
    }
    return state;
}

int main() {
    DDRD = 0x00; PORTD = 0xFF;
    DDRC = 0xFF; PORTC = 0x00;
    DDRB = 0xFF; PORTB = 0x00;
    adc_init();
    nrf24_init();
    nrf24_config(6, 4);
    nrf24_tx_address(_tx_address);
    nrf24_rx_address(_rx_address);

    // Put motor to sleep on startup
    PORTC = SetBit(PORTC, SLEEP_PIN, 0);
    // Close  window so we know what state it is in for sure
    window_close();

    _temp_in = therm_read_temperature(1);
    _temp_out = therm_read_temperature(0);

    /* define tasks */
    tasksNum = 4; // declare number of tasks
    task tsks[4]; // initialize the task array
    tasks = tsks; // set the task array

    uint8_t i = 0;
    tasks[i].state = TEMP_GET;
    tasks[i].period = 3000;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_temp;
    i++;
    tasks[i].state = NRF_SEND;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_nrf;
    i++;
    tasks[i].state = WAIT;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_alert;
    i++;
    tasks[i].state = AUTO_OFF;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_auto;

    TimerSet(100);
    TimerOn();

    while(1) {}
}
