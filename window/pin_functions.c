/*
* ----------------------------------------------------------------------------
* “THE COFFEEWARE LICENSE” (Revision 1):
* <ihsan@kehribar.me> wrote this file. As long as you retain this notice you
* can do whatever you want with this stuff. If we meet some day, and you think
* this stuff is worth it, you can buy me a coffee in return.
* -----------------------------------------------------------------------------
* Please define your platform spesific functions in this file ...
* -----------------------------------------------------------------------------
*/

#include <avr/io.h>

#define set_bit(reg,bit) reg |= (1<<bit)
#define clr_bit(reg,bit) reg &= ~(1<<bit)
#define check_bit(reg,bit) (reg&(1<<bit))

#define RF_PORT PORTA
#define RF_PIN  PINA
#define RF_DDR  DDRA
#define CE      1
#define CSN     2
#define SCK     3
#define MOSI    4
#define MISO    5

/* ------------------------------------------------------------------------- */
void nrf24_setupPins()
{
    set_bit(RF_DDR,CE); // CE output
    set_bit(RF_DDR,CSN); // CSN output
    set_bit(RF_DDR,SCK); // SCK output
    set_bit(RF_DDR,MOSI); // MOSI output
    clr_bit(RF_DDR,MISO); // MISO input
}
/* ------------------------------------------------------------------------- */
void nrf24_ce_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(RF_PORT,CE);
    }
    else
    {
        clr_bit(RF_PORT,CE);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_csn_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(RF_PORT,CSN);
    }
    else
    {
        clr_bit(RF_PORT,CSN);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_sck_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(RF_PORT, SCK);
    }
    else
    {
        clr_bit(RF_PORT,SCK);
    }
}
/* ------------------------------------------------------------------------- */
void nrf24_mosi_digitalWrite(uint8_t state)
{
    if(state)
    {
        set_bit(RF_PORT,MOSI);
    }
    else
    {
        clr_bit(RF_PORT,MOSI);
    }
}
/* ------------------------------------------------------------------------- */
uint8_t nrf24_miso_digitalRead()
{
    return check_bit(RF_PIN,MISO);
}
/* ------------------------------------------------------------------------- */
