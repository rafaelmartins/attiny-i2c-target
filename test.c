/*
 * attiny-i2c-target: An implementation of I2C target for Attiny
 *                    microcontrollers, using USI.
 * Copyright (C) 2019-2020 Rafael G. Martins <rafael@rafaelmartins.eng.br>
 *
 * This program can be distributed under the terms of the BSD License.
 * See the file LICENSE.
 */

#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2c-target.h"


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

    i2c_target_init(0x20);
    i2c_target_add_register(handler);

    sei();
    while (true);

    return 0;
}
