/*
 * attiny-i2c-device: An implementation of I2C-connected device for Attiny
 *                    microcontrollers, using USI.
 * Copyright (C) 2019 Rafael G. Martins <rafael@rafaelmartins.eng.br>
 *
 * This program can be distributed under the terms of the BSD License.
 * See the file LICENSE.
 */

#pragma once

#include <stdint.h>

#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#   define DDR_I2C DDRA
#   define PORT_I2C PORTA
#   define PIN_I2C PINA
#   define SDA_I2C 6
#   define SCL_I2C 4
#   define TIMSK_I2C TIMSK0
#   define TIMER_VECT_I2C TIM0_COMPA_vect
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#   define DDR_I2C DDRB
#   define PORT_I2C PORTB
#   define PIN_I2C PINB
#   define SDA_I2C 0
#   define SCL_I2C 2
#   define TIMSK_I2C TIMSK
#   define TIMER_VECT_I2C TIMER0_COMPA_vect
#else
#   error "Unsupported Attiny microcontroller."
#endif

#ifndef I2C_REGISTER_ALLOC
#   define I2C_REGISTER_ALLOC 10
#endif

typedef void (*i2c_device_handler_func_t) (uint8_t reg, uint8_t val);

void i2c_device_init(uint8_t addr);
void i2c_device_add_register(i2c_device_handler_func_t func);
void i2c_device_set_register(uint8_t reg, uint8_t val);
