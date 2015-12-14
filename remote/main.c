/**
 * Author: James Hollister
 * Partner: Roberto Pasillas
 *
 * Main state machine functionality for remote
 */
#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include "nrf24.h"
#include "lcd.h"
#include "scheduler.h"
#include "bit.h"

#define DEG_SYM 0xDF

#define OPEN_BTN  7
#define CLOSE_BTN 6
#define SET_BTN   5

#define NO_CONN 0
#define CLOSED  1
#define OPEN    2
#define CLOSING 3
#define OPENING 4
#define OPEN_PARTIAL 5

static uint8_t _tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
static uint8_t _rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
static int8_t _rcv_buffer[4];
static int8_t _send_buffer[4];
static int8_t _temp_in = 0;
static int8_t _temp_out = 0;
static int8_t _temp_max = 0xFF;
static int8_t _temp_min = 65;
static uint8_t _status = 0;
static uint8_t _auto_set = 0;
static uint8_t _auto_send = 0;
static uint8_t _min_set = 0;
static uint8_t _max_set = 0;
static uint8_t _auto = 0;
static uint8_t _data_rcvd = 0;
static uint8_t _rf_output = 0;

int send_rx(uint8_t *buffer) {
    uint8_t result;
    nrf24_send(buffer);
    while (!nrf24_isSending());

    result = nrf24_retransmissionCount();
    nrf24_powerUpRx();
    return result;
}

/* Updates display using the current received temperatures */
void update_display(void) {
    static char temp[5];
    uint8_t cursor = 1;
    LCD_ClearScreen();
    LCD_DisplayString(cursor, "in:");
    cursor += 3;
    if (_data_rcvd) {
        itoa(_temp_in, temp, 10);
        LCD_DisplayString(cursor, temp);
        cursor += strlen(temp);
        LCD_Cursor(cursor);
        LCD_WriteData(DEG_SYM);
    }
    else {
        LCD_DisplayString(cursor, "--");
    }
    cursor = 9;
    LCD_DisplayString(cursor, "out:");
    cursor += 4;
    if (_data_rcvd) {
        itoa(_temp_out, temp, 10);
        LCD_DisplayString(cursor, temp);
        cursor += strlen(temp);
        LCD_Cursor(cursor);
        LCD_WriteData(DEG_SYM);
    }
    else {
        LCD_DisplayString(cursor, "--");
    }
    cursor = 17;
    switch (_status) {
        case NO_CONN:
            LCD_DisplayString(cursor, "no conn");
            break;
        case CLOSED:
            LCD_DisplayString(cursor, "closed");
            break;
        case OPEN:
            LCD_DisplayString(cursor, "open");
            break;
        case OPENING:
            LCD_DisplayString(cursor, "opening");
            break;
        case CLOSING:
            LCD_DisplayString(cursor, "closing");
            break;
        case OPEN_PARTIAL:
            LCD_DisplayString(cursor, "open");
            break;
        default:
            LCD_DisplayString(cursor, "error");
            break;
    }
    cursor = 25;
    LCD_DisplayString(cursor, "auto:");
    cursor += 5;
    if (_auto) {
        LCD_DisplayString(cursor, "on");
    }
    else {
        LCD_DisplayString(cursor, "off");
    }

    LCD_Cursor(0);
}

/* Update the display if any of the state variables used in the
 * display are updated
 */
enum disp_states { DISP_DEF, DISP_MIN_SET, DISP_MAX_SET };
int tick_disp(int state) {
    static uint8_t prev_status;
    static uint8_t prev_auto;
    static int8_t prev_in;
    static int8_t prev_out;

    static int8_t prev_temp;
    static char temp[5];
    switch (state) {
        case DISP_DEF:
            if (_min_set) {
                state = DISP_MIN_SET;
                LCD_ClearScreen();
                LCD_DisplayString(1, "min temp:");
                itoa(_temp_min, temp, 10);
                LCD_DisplayString(10, temp);
                LCD_Cursor(0);
            }

            else if (prev_status != _status ||
                    prev_auto != _auto ||
                    prev_in != _temp_in ||
                    prev_out != _temp_out) {
                prev_status = _status;
                prev_auto = _auto;
                prev_in = _temp_in;
                prev_out = _temp_out;
                update_display();
            }
            break;
        case DISP_MIN_SET:
            if (_max_set) {
                prev_temp = _temp_max;
                state = DISP_MAX_SET;
                LCD_ClearScreen();
                LCD_DisplayString(1, "max temp:");
                itoa(_temp_max, temp, 10);
                LCD_DisplayString(10, temp);
                LCD_Cursor(0);
            }
            else {
                itoa(_temp_min, temp, 10);
                LCD_DisplayString(10, temp);
                LCD_Cursor(0);
            }
            break;
        case DISP_MAX_SET:
            if (!_max_set) {
                update_display();
                state = DISP_DEF;
            }
            else {
                itoa(_temp_max, temp, 10);
                LCD_DisplayString(10, temp);
                LCD_Cursor(0);
            }
            break;
        default:
            prev_status = _status;
            prev_auto = _auto;
            prev_in = _temp_in;
            prev_out = _temp_out;
            state = DISP_DEF;
            break;
    }
    return state;
}


/*
 * Handle input from the three buttons
 */
enum input_states { IN_WAIT, IN_CLOSE, IN_OPEN, IN_SET, IN_SET_MIN, IN_SET_MAX };
int tick_menu(int state) {
    switch (state) {
        case IN_WAIT:
            if ( !GetBit(PINC, SET_BTN) ) {
                _min_set = 1;
                state = IN_SET;
            }
            break;
        case IN_SET:
            if ( GetBit(PINC, SET_BTN)  && _min_set) {
                state = IN_SET_MIN;
            }
            else if ( GetBit(PINC, SET_BTN)  && _max_set) {
                state = IN_SET_MAX;
            }
            else if ( GetBit(PINC, SET_BTN)  && _auto_set) {
                _send_buffer[0] = 3;
                _send_buffer[1] = _temp_max;
                _send_buffer[2] = _temp_min;
                send_rx(_send_buffer);
                state = IN_WAIT;
            }
            break;
        case IN_SET_MIN:
            if ( !GetBit(PINC, SET_BTN) ) {
                if (_temp_max == -1) {
                    _temp_max = _temp_min + 5;
                }
                _min_set = 0;
                _max_set = 1;
                state = IN_SET;
            }
            else if ( !GetBit(PINC, CLOSE_BTN) ) {
                _temp_min = _temp_min < 100 ? _temp_min + 1 : _temp_min;
                state = IN_SET;
            }
            else if ( !GetBit(PINC, OPEN_BTN) ) {
                _temp_min = _temp_min > 0 ? _temp_min - 1 : _temp_min;
                state = IN_SET;
            }
            break;
        case IN_SET_MAX:
            if ( !GetBit(PINC, SET_BTN) ) {
                _max_set = 0;
                _auto_set = 1;
                state = IN_SET;
            }
            else if ( !GetBit(PINC, CLOSE_BTN) ) {
                _temp_max = _temp_max < 110 ? _temp_max + 1 : _temp_max;
                state = IN_SET;
            }
            else if ( !GetBit(PINC, OPEN_BTN) ) {
                _temp_max = _temp_max > (_temp_min + 5)  ? _temp_max - 1 : _temp_max;
                state = IN_SET;
            }
            break;
        default:
            state = IN_WAIT;
            break;
    }
    return state;
}

int tick_btn(int state) {
    static uint8_t temp;
    switch (state) {
        case IN_WAIT:
            if ( !GetBit(PINC, OPEN_BTN)  && !_min_set && !_max_set) {
                state = IN_SET;
                _send_buffer[0] = OPEN;
                if (send_rx(_send_buffer) == NRF24_MESSAGE_LOST) {
                    _status = NO_CONN;
                }
            }
            else if ( !GetBit(PINC, CLOSE_BTN) && !_min_set && !_max_set) {
                state = IN_SET;
                _send_buffer[0] = CLOSED;
                if (send_rx(_send_buffer) == NRF24_MESSAGE_LOST) {
                    _status = NO_CONN;
                }
            }
            break;
        case IN_SET:
            if (GetBit(PINC, OPEN_BTN)) {
                state = IN_WAIT;
            }
            break;
        default:
            state = IN_WAIT;
            break;
    }
    return state;
}




enum nrf_states { NRF_RCV, NRF_SEND, NRF_WAIT };

int tick_nrf(int state) {
    switch(state) {
        case NRF_RCV:
            if (nrf24_dataReady()) {
                _data_rcvd = 1;
                nrf24_getData(_rcv_buffer);
                _temp_in = _rcv_buffer[0];
                _temp_out = _rcv_buffer[1];
                _status = _rcv_buffer[2];
                _auto = _rcv_buffer[3];
            }
            break;
        default:
            state = NRF_RCV;
            break;
    }
    return state;
}


int main() {
    /* initialize lcd data and contorl ports */
    DDRD = 0xFF; PORTD = 0;
    DDRC = 0x1F; PORTC = 0xE0;

    LCD_init();
    update_display();


    /* Channel #2, payload length: 4 */
    nrf24_init();
    nrf24_config(6, 4);

    /* Set the device addresses */
    nrf24_tx_address(_tx_address);
    nrf24_rx_address(_rx_address);

    /* define tasks */
    tasksNum = 4; // declare number of tasks
    task tsks[4]; // initialize the task array
    tasks = tsks; // set the task array

    uint8_t i = 0;
    tasks[i].state = NRF_RCV;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_nrf;
    i++;
    tasks[i].state = -1;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_disp;
    i++;
    tasks[i].state = IN_WAIT;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_btn;
    i++;
    tasks[i].state = IN_WAIT;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_menu;

    TimerSet(100);
    TimerOn();

    while(1) {}
}
