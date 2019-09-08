/*
 * attiny-i2c-device: An implementation of I2C-connected device for Attiny
 *                    microcontrollers, using USI.
 * Copyright (C) 2019 Rafael G. Martins <rafael@rafaelmartins.eng.br>
 *
 * This program can be distributed under the terms of the BSD License.
 * See the file LICENSE.
 */

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "i2c-device.h"

typedef struct {
    uint8_t data;
    bool set;
    i2c_device_handler_func_t func;
} i2c_register_t;

typedef enum {
    ADDR = 1,
    SET_RX,
    RX,
    SET_TX,
    TX,
    CHECK_TX,
} i2c_state_t;

static uint8_t address;
static size_t reg_counter;
static i2c_register_t regs[I2C_REGISTER_ALLOC];

static volatile i2c_state_t state;
static volatile bool reg_set;
static volatile uint8_t reg;


void
i2c_device_init(uint8_t addr)
{
    reg_counter = 0;

    address = addr;

    DDR_I2C |= (1 << SCL_I2C) | (1 << SDA_I2C);
    PORT_I2C |= (1 << SCL_I2C) | (1 << SDA_I2C);
    DDR_I2C &= ~(1 << SDA_I2C);

    USICR = (1 << USISIE) | (1 << USIWM1) | (1 << USICS1);
    USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

    TIMSK_I2C |= (1 << OCIE0A);
    TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0) | (1 << WGM01) | (1 << WGM00));
    TCCR0B &= ~((1 << FOC0A) | (1 << WGM02) | (1 << CS01));
    TCCR0B |= (1 << CS02) | (1 << CS00);  // prescale 1024
}


void
i2c_device_add_register(i2c_device_handler_func_t func)
{
    if (reg_counter >= I2C_REGISTER_ALLOC)
        return;
    regs[reg_counter].data = 0;
    regs[reg_counter].set = false;
    regs[reg_counter].func = func;
    reg_counter++;
}


void
i2c_device_set_register(uint8_t reg, uint8_t val)
{
    if (reg >= reg_counter)
        return;
    regs[reg].data = val;
    regs[reg].set = true;
}


ISR(USI_STR_VECT_I2C) {
    state = ADDR;

    DDR_I2C &= ~(1 << SDA_I2C);

    while ((PIN_I2C & (1 << SCL_I2C)) && !((PIN_I2C & (1 << SDA_I2C))));

    if (!(PIN_I2C & (1 << SDA_I2C)))
        USICR = (1 << USISIE) | (1 << USIOIE) | (1 << USIWM1) | (1 << USIWM0) |
            (1 << USICS1);
    else
        USICR = (1 << USISIE) | (1 << USIWM1) | (1 << USICS1);

    USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);
}


ISR(USI_OVF_VECT_I2C) {
    switch (state) {

        case ADDR:
            // USIDR:
            //  - LSB: 1 (master reading), 0 (master writing)
            //  - other bits: address

            // we only want data sent directly to us, no broadcasts. this may
            // sound a bit odd, but we most likely won't know what to do with
            // broadcasted data anyway.
            if ((USIDR >> 1) == address) {
                state = (USIDR & (1 << 0)) ? SET_TX : SET_RX;

                // set to send ack
                USIDR = 0;
                DDR_I2C |= (1 << SDA_I2C);
                USISR = (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) |
                    (1 << USICNT3) | (1 << USICNT2) | (1 << USICNT1);

                break;
            }

            // set start condition
            USICR = (1 << USISIE) | (1 << USIWM1) | (1 << USICS1);
            USISR = (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

            break;

        case SET_RX:
            state = RX;

            // set read data
            DDR_I2C &= ~(1 << SDA_I2C);
            USISR = (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

            break;

        case RX:
            state = SET_RX;

            if (reg_set) {
                regs[reg].data = USIDR;
                regs[reg].set = true;
                reg = 0;
                reg_set = false;
            }
            else {
                if (USIDR >= reg_counter) {

                    // set start condition
                    USICR = (1 << USISIE) | (1 << USIWM1) | (1 << USICS1);
                    USISR = (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

                    break;
                }

                reg_set = true;
                reg = USIDR;
            }

            // set send ack
            USIDR = 0;
            DDR_I2C |= (1 << SDA_I2C);
            USISR = (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | (1 << USICNT3) |
                (1 << USICNT2) | (1 << USICNT1);

            break;

        case CHECK_TX:
            // any data in USIDR means we got an ack
            if (USIDR) {
                reg_set = false;
                reg = 0;

                // set start condition
                USICR = (1 << USISIE) | (1 << USIWM1) | (1 << USICS1);
                USISR = (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

                break;
            }

            // no ack, just fallthrough to re-send data

        case SET_TX:
            if (!reg_set) {

                // set start condition
                USICR = (1 << USISIE) | (1 << USIWM1) | (1 << USICS1);
                USISR = (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

                break;
            }

            state = TX;

            USIDR = regs[reg].data;

            reg_set = false;
            reg = 0;

            // set send data
            DDR_I2C |= (1 << SDA_I2C);
            USISR = (1 << USIOIF) | (1 << USIPF) | (1 << USIDC);

            break;

        case TX:
            state = CHECK_TX;

            // set read ack
            DDR_I2C &= ~(1 << SDA_I2C);
            USIDR = 0;
            USISR = (1 << USIOIF) | (1 << USIPF) | (1 << USIDC) | (1 << USICNT3) |
                (1 << USICNT2) | (1 << USICNT1);

            break;
    }
}


ISR(TIMER_VECT_I2C) {
    for (size_t i = 0; i < reg_counter; i++) {
        if (!regs[i].set)
            continue;
        regs[i].func(i, regs[i].data);
        regs[i].set = false;
    }
}
