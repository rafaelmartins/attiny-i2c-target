/*
 * attiny-i2c-device: An implementation of I2C-connected device for Attiny
 *                    microcontrollers, using USI.
 * Copyright (C) 2019 Rafael G. Martins <rafael@rafaelmartins.eng.br>
 *
 * This program can be distributed under the terms of the BSD License.
 * See the file LICENSE.
 */

#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2c-device.h"


void
handler(uint8_t reg, uint8_t val)
{
    (void) reg;

    if (val)
        PORTB |= (1 << 0);
    else
        PORTB &= ~(1 << 0);
}


int
main(void)
{
    DDRB |= (1 << 0);

    i2c_device_init(0x20);
    i2c_device_add_handler(handler);

    sei();
    while (true);

    return 0;
}
