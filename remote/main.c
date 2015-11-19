#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include "nrf24.h"
#include "lcd.h"
#include "adc.h"
#include "scheduler.h"

#define DEG_SYM 0xDF

enum window_status {
    NO_CONN,
    CLOSED,
    OPEN,
    CLOSING,
    OPENING
};

static uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
static uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
static int8_t rcv_buffer[3];
static int8_t send_buffer[3];
static int8_t temp_in = 0;
static int8_t temp_out = 0;
static uint8_t status = 0;
static uint8_t auto_set = 1;
static uint8_t data_rcvd = 0;

/* Updates display using the current received temperatures */
void update_display(void) {
    static char temp[5];
    uint8_t cursor = 1;
    LCD_ClearScreen();
    LCD_DisplayString(cursor, "in:");
    cursor += 3;
    if (data_rcvd) {
        itoa(temp_in, temp, 10);
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
    if (data_rcvd) {
        itoa(temp_out, temp, 10);
        LCD_DisplayString(cursor, temp);
        cursor += strlen(temp);
        LCD_Cursor(cursor);
        LCD_WriteData(DEG_SYM);
    }
    else {
        LCD_DisplayString(cursor, "--");
    }
    cursor = 17;
    switch (status) {
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
        default:
            LCD_DisplayString(cursor, "error");
            break;
    }
    cursor = 25;
    LCD_DisplayString(cursor, "auto:");
    cursor += 5;
    if (auto_set) {
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
enum disp_states { DISP };
int tick_disp(int state) {
    static uint8_t prev_status, prev_auto;
    static int8_t prev_in, prev_out;
    switch (state) {
        case DISP:
            if (prev_status != status ||
                    prev_auto != auto_set ||
                    prev_in != temp_in ||
                    prev_out != temp_out) {
                prev_status = status;
                prev_auto = auto_set;
                prev_in = temp_in;
                prev_out = temp_out;
                update_display();
            }
            break;
        default:
            prev_status = status;
            prev_auto = auto_set;
            prev_in = temp_in;
            prev_out = temp_out;
            state = DISP;
            break;
    }
    return state;
}



enum rcv_states { RCV_WAIT };
int tick_rcv(int state) {
    switch (state) {
        case RCV_WAIT:
            if (nrf24_dataReady()) {
                data_rcvd = 1;
                nrf24_getData(rcv_buffer);
                temp_in = rcv_buffer[0];
                temp_out = rcv_buffer[1];
                status = rcv_buffer[2];
            }
            break;
        default:
            state = RCV_WAIT;
            break;
    }
    return state;
}



int main() {
    /* initialize lcd data and contorl ports */
    DDRD = 0xFF; PORTD = 0;
    DDRC = 0xFF; PORTC = 0;

    LCD_init();
    update_display();


    /* Channel #2, payload length: 3 */
    nrf24_init();
    nrf24_config(2, 3);

    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);

    /* initialize adc and set to A6 */
    adc_init();
    adc_set_pin(6);

    /* define tasks */
    tasksNum = 2; // declare number of tasks
    task tsks[2]; // initialize the task array
    tasks = tsks; // set the task array

    uint8_t i = 0;
    tasks[i].state = -1;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_rcv;
    i++;
    tasks[i].state = -1;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &tick_disp;

    TimerSet(100);
    TimerOn();

    while(1) {}
}
