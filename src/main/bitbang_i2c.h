#ifndef BITBANG_I2C_H
#define BITBANG_I2C_H

#include <stdint.h>

void gpio_set(uint32_t pin);
void gpio_clear(uint32_t pin);
void gpio_enable(uint32_t pin);

#endif