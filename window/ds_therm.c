/**
 * Driver implementation for ds18b20 digital temperature sensor.
 * Adapted from: http://teslabs.com/openplayer/docs/docs/other/ds18b20_pre1.pdf
 */
#include "ds18b20.h"


inline __attribute__((gnu_inline)) void therm_delay(uint16_t delay) {
    while (delay--) asm volatile("nop");
}

uint8_t therm_reset(uint8_t pin) {
    uint8_t i;
    // Pull line low and wait for 480us
    THERM_LOW(pin);
    THERM_OUTPUT_MODE(pin);
    therm_delay(us(480));

    // Release line and wait for 60 us
    THERM_INPUT_MODE(pin);
    therm_delay(us(60));

    // Store line value and wait until the completion of 480us period
    i = (THERM_PIN & (1 << pin));
    therm_delay(us(420));

    // Return the value read from the presence pulse
    return i;
}

void therm_write_bit(uint8_t bit, uint8_t pin) {
    // pull line low for 1 us
    THERM_LOW(pin);
    THERM_OUTPUT_MODE(pin);
    therm_delay(us(1));

    if (bit) THERM_INPUT_MODE(pin);

    therm_delay(us(60));
    THERM_INPUT_MODE(pin);
}

uint8_t therm_read_bit(uint8_t pin) {
    uint8_t bit = 0;

    // Pull line low for 1 us
    THERM_LOW(pin);
    THERM_OUTPUT_MODE(pin);
    therm_delay(us(1));

    // Release the line adn wait for 14 us
    THERM_INPUT_MODE(pin);
    therm_delay(us(14));

    // Reade line value
    if (THERM_PIN & (1 << pin)) bit = 1;

    // Wait for 45 us to end and return read value
    therm_delay(us(45));
    return bit;
}

uint8_t therm_read_byte(uint8_t pin) {
    uint8_t i = 8, n = 0;
    while(i--) {
        n >>= 1;
        n |= (therm_read_bit(pin) << 7);
    }
    return n;
}


void therm_write_byte(uint8_t byte, uint8_t pin) {
    uint8_t i = 8;
    while (i--) {
        // write actual bit and shift one position right to make bit ready
        therm_write_bit(byte & 1, pin);
        byte >>= 1;
    }
}

int8_t therm_read_temperature(uint8_t pin) {
    // Buffer length must be at least 12 bytes long
    uint8_t temperature[2];
    int8_t digit;
    uint16_t decimal;

    // Reset, skip ROM and start temperature conversion
    therm_reset(pin);
    therm_write_byte(THERM_CMD_SKIPROM, pin);
    therm_write_byte(THERM_CMD_CONVERTTEMP, pin);
    // wait until conversion is complete
    while (!therm_read_bit(pin));
    // reset, skip ROM and send command to read scratchpad
    therm_reset(pin);
    therm_write_byte(THERM_CMD_SKIPROM, pin);
    therm_write_byte(THERM_CMD_RSCRATCHPAD, pin);

    // read scratchpad (only 2 first bytes)
    temperature[0] = therm_read_byte(pin);
    temperature[1] = therm_read_byte(pin);
    therm_reset(pin);

    // Store temperature integer digits and decimal digits
    digit = temperature[0] >> 4;
    digit |= (temperature[1] & 0x07) << 4;
    // Store decimal digits
    decimal = temperature[0] & 0x0F;
    decimal *= THERM_DECIMAL_STEPS_12BIT;

    if (decimal >= 5000) {
        digit += 1;
    }

    return (digit * 9 / 5) + 32;
}
